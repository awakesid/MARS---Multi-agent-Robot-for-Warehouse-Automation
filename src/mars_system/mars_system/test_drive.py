import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_ID      = 1
BOT_IP      = '192.168.1.16'   # change to your ESP32 IP
UDP_PORT    = 4210
TARGET_LOAD = 3                  # load ID 33 → index 3

ARRIVE_DIST  = 80    # mm — close enough to target
ANGLE_THRESH = 15    # degrees — acceptable heading error
FILTER_N     = 8     # camera readings to average

# ─── STATES ──────────────────────────────────────────────────────
WAITING    = 'WAITING'
TURNING    = 'TURNING'
DRIVING    = 'DRIVING'
CHECKING   = 'CHECKING'
ARRIVED    = 'ARRIVED'

# ─── PHASES ──────────────────────────────────────────────────────
PHASE_TO_LOAD = 'TO_LOAD'
PHASE_TO_DROP = 'TO_DROP'


# ─── FILTERS ─────────────────────────────────────────────────────
class PoseFilter:
    def __init__(self, n=8):
        self.xs = deque(maxlen=n)
        self.ys = deque(maxlen=n)

    def update(self, x, y):
        self.xs.append(x)
        self.ys.append(y)

    def get(self):
        return sum(self.xs)/len(self.xs), sum(self.ys)/len(self.ys)

    def ready(self, min_n=5):
        return len(self.xs) >= min_n

    def clear(self):
        self.xs.clear()
        self.ys.clear()


class YawFilter:
    def __init__(self, n=8):
        self.yaws = deque(maxlen=n)

    def update(self, yaw):
        self.yaws.append(yaw)

    def get(self):
        sins = sum(math.sin(math.radians(y)) for y in self.yaws)
        coss = sum(math.cos(math.radians(y)) for y in self.yaws)
        return math.degrees(math.atan2(sins, coss))

    def ready(self, min_n=5):
        return len(self.yaws) >= min_n

    def clear(self):
        self.yaws.clear()


# ─── MAIN NODE ───────────────────────────────────────────────────
class TestDrive(Node):
    def __init__(self):
        super().__init__('test_drive')

        self.bot_filter   = PoseFilter(FILTER_N)
        self.bot_yaw      = YawFilter(FILTER_N)
        self.load_filter  = PoseFilter(FILTER_N)
        self.drop_filter  = PoseFilter(FILTER_N)

        self.state        = WAITING
        self.phase        = PHASE_TO_LOAD
        self.busy         = False
        self.settle_ticks = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)

        self.create_subscription(String, '/bot_poses', self.bots_cb,  10)
        self.create_subscription(String, '/loads',     self.loads_cb, 10)
        self.create_subscription(String, '/dropzone',  self.drop_cb,  10)
        self.create_timer(0.2, self.loop)

        self.get_logger().info('──────────────────────────────────')
        self.get_logger().info(f' Bot{BOT_ID} → Load{TARGET_LOAD} → DropZone')
        self.get_logger().info(f' ESP32: {BOT_IP}:{UDP_PORT}')
        self.get_logger().info('──────────────────────────────────')

    # ── Subscribers ───────────────────────────────────────────────
    def bots_cb(self, msg):
        for b in json.loads(msg.data):
            if b['id'] == BOT_ID:
                self.bot_filter.update(b['x_mm'], b['y_mm'])
                self.bot_yaw.update(b['yaw_deg'])

    def loads_cb(self, msg):
        for l in json.loads(msg.data):
            if l['id'] == TARGET_LOAD:
                self.load_filter.update(l['x_mm'], l['y_mm'])

    def drop_cb(self, msg):
        d = json.loads(msg.data)
        self.drop_filter.update(d['x_mm'], d['y_mm'])

    # ── UDP ───────────────────────────────────────────────────────
    def send(self, cmd: dict):
        cmd['id'] = BOT_ID
        self.sock.sendto(json.dumps(cmd).encode(), (BOT_IP, UDP_PORT))
        self.busy = True
        self.get_logger().info(f'TX → {cmd}')

    def recv(self):
        try:
            data, _ = self.sock.recvfrom(512)
            d = json.loads(data.decode())
            if d.get('type') == 'done':
                self.busy = False
                self.get_logger().info('RX ← done')
        except socket.timeout:
            pass

    # ── Get current target pose ───────────────────────────────────
    def get_target(self):
        if self.phase == PHASE_TO_LOAD:
            if not self.load_filter.ready():
                return None
            return self.load_filter.get()
        else:
            if not self.drop_filter.ready():
                return None
            return self.drop_filter.get()

    # ── Compute errors ────────────────────────────────────────────
    def get_error(self, tx, ty):
        bx, by = self.bot_filter.get()
        yaw    = self.bot_yaw.get()
        dx = tx - bx
        dy = ty - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = target_angle - yaw
        angle_err    = (angle_err + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    # ── Reset filters for fresh readings ─────────────────────────
    def reset_filters(self):
        self.bot_filter.clear()
        self.bot_yaw.clear()

    # ── Control loop ──────────────────────────────────────────────
    def loop(self):
        self.recv()

        if self.state == ARRIVED and self.phase == PHASE_TO_DROP:
            return  # fully done

        # Check bot is visible
        if not self.bot_filter.ready() or not self.bot_yaw.ready():
            self.get_logger().warn('Waiting for Bot1 marker...')
            return

        # Check target is visible
        target = self.get_target()
        if target is None:
            label = 'Load' if self.phase == PHASE_TO_LOAD else 'DropZone'
            self.get_logger().warn(f'Waiting for {label} marker...')
            return

        if self.busy:
            return

        tx, ty = target
        dist, angle_err, bx, by, yaw = self.get_error(tx, ty)

        self.get_logger().info(
            f'[{self.phase}][{self.state}] '
            f'bot=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
            f'target=({tx:.0f},{ty:.0f}) '
            f'dist={dist:.0f}mm err={angle_err:.0f}°')

        # ── CHECKING: wait for camera to settle ───────────────────
        if self.state == CHECKING:
            self.settle_ticks += 1
            if self.settle_ticks < 5:
                return
            self.reset_filters()
            self.state = WAITING
            return

        # ── WAITING: accumulate fresh readings ────────────────────
        if self.state == WAITING:
            if not self.bot_filter.ready(min_n=FILTER_N):
                return
            self.state = TURNING
            return

        # ── ARRIVED: switch phase or finish ───────────────────────
        if dist < ARRIVE_DIST:
            if self.phase == PHASE_TO_LOAD:
                self.get_logger().info('=== REACHED LOAD — going to DropZone ===')
                self.phase        = PHASE_TO_DROP
                self.state        = WAITING
                self.settle_ticks = 0
                self.reset_filters()
                return
            else:
                self.get_logger().info('=== ARRIVED AT DROP ZONE ===')
                self.send({'cmd': 'STOP'})
                self.state = ARRIVED
                return

        # ── TURNING ───────────────────────────────────────────────
        if self.state == TURNING:
            if abs(angle_err) > ANGLE_THRESH:
                if angle_err > 0:
                    self.send({'cmd': 'TURN_L', 'angle': round(abs(angle_err), 1)})
                else:
                    self.send({'cmd': 'TURN_R', 'angle': round(abs(angle_err), 1)})
            self.state = DRIVING
            return

        # ── DRIVING ───────────────────────────────────────────────
        if self.state == DRIVING:
            drive_dist = max(dist - ARRIVE_DIST + 20, 50)
            self.send({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            self.settle_ticks = 0
            self.state = CHECKING


def main(args=None):
    rclpy.init(args=args)
    node = TestDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.sendto(
            json.dumps({'cmd': 'STOP', 'id': BOT_ID}).encode(),
            (BOT_IP, UDP_PORT))
        node.destroy_node()
        rclpy.shutdown()