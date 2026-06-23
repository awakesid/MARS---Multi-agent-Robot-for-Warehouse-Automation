import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_ID      = 1
BOT_IP      = '192.168.1.16'
UDP_PORT    = 4210
TARGET_LOAD = 3

ARRIVE_DIST  = 80    # mm
ANGLE_THRESH = 15    # degrees — correction threshold after driving
FILTER_N     = 8     # more readings for better accuracy before planning

# ─── STATE MACHINE ───────────────────────────────────────────────
WAITING   = 'WAITING'
TURNING   = 'TURNING'
DRIVING   = 'DRIVING'
CHECKING  = 'CHECKING'   # wait for camera to settle after move
CORRECTING= 'CORRECTING'
ARRIVED   = 'ARRIVED'


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


class TestDrive(Node):
    def __init__(self):
        super().__init__('test_drive')

        self.bot_filter  = PoseFilter(FILTER_N)
        self.bot_yaw     = YawFilter(FILTER_N)
        self.load_filter = PoseFilter(FILTER_N)

        self.state      = WAITING
        self.busy       = False
        self.settle_ticks = 0   # count loops after move to let camera settle

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)

        self.create_subscription(String, '/bot_poses', self.bots_cb,  10)
        self.create_subscription(String, '/loads',     self.loads_cb, 10)
        self.create_timer(0.2, self.loop)   # 5Hz

        self.get_logger().info(f'PlanExecute Bot{BOT_ID} → Load{TARGET_LOAD}')

    def bots_cb(self, msg):
        for b in json.loads(msg.data):
            if b['id'] == BOT_ID:
                self.bot_filter.update(b['x_mm'], b['y_mm'])
                self.bot_yaw.update(b['yaw_deg'])

    def loads_cb(self, msg):
        for l in json.loads(msg.data):
            if l['id'] == TARGET_LOAD:
                self.load_filter.update(l['x_mm'], l['y_mm'])

    def send(self, cmd):
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
                self.get_logger().info(f'RX ← done')
        except socket.timeout:
            pass

    def get_error(self):
        bx, by = self.bot_filter.get()
        lx, ly = self.load_filter.get()
        yaw    = self.bot_yaw.get()
        dx = lx - bx
        dy = ly - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = target_angle - yaw
        angle_err    = (angle_err + 180) % 360 - 180
        return dist, angle_err, bx, by, lx, ly, yaw

    def loop(self):
        self.recv()

        if self.state == ARRIVED:
            return

        if not (self.bot_filter.ready() and
                self.load_filter.ready() and
                self.bot_yaw.ready()):
            self.get_logger().warn('Waiting for markers...')
            return

        if self.busy:
            return

        dist, angle_err, bx, by, lx, ly, yaw = self.get_error()

        self.get_logger().info(
            f'[{self.state}] '
            f'bot=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
            f'load=({lx:.0f},{ly:.0f}) '
            f'dist={dist:.0f}mm err={angle_err:.0f}°')

        # ── ARRIVED ───────────────────────────────────────────────
        if dist < ARRIVE_DIST:
            self.get_logger().info('=== ARRIVED ===')
            self.send({'cmd': 'STOP'})
            self.state = ARRIVED
            return

        # ── CHECKING: camera settling after a move ────────────────
        if self.state == CHECKING:
            self.settle_ticks += 1
            if self.settle_ticks < 5:   # wait 5 loops = 1 second
                return
            # Camera settled — clear old readings, take fresh ones
            self.bot_filter.xs.clear(); self.bot_filter.ys.clear()
            self.bot_yaw.clear()
            self.state = WAITING
            return

        # ── WAITING: accumulate fresh readings then plan ──────────
        if self.state == WAITING:
            if not self.bot_filter.ready(min_n=FILTER_N):
                return   # keep accumulating
            self.get_logger().info('── PLANNING ──')
            self.state = TURNING
            return

        # ── TURNING: execute planned turn in one shot ─────────────
        if self.state == TURNING:
            if abs(angle_err) > ANGLE_THRESH:
                # Turn full angle at once
                if angle_err > 0:
                    self.send({'cmd': 'TURN_L', 'angle': round(abs(angle_err), 1)})
                else:
                    self.send({'cmd': 'TURN_R', 'angle': round(abs(angle_err), 1)})
                self.state = DRIVING   # after turn, drive
            else:
                self.get_logger().info('Heading ok — skipping turn')
                self.state = DRIVING

        # ── DRIVING: drive full distance in one shot ──────────────
        elif self.state == DRIVING:
            if self.busy:
                return
            drive_dist = max(dist - ARRIVE_DIST + 20, 50)
            self.send({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            # After driving, wait for camera to settle then check
            self.settle_ticks = 0
            self.state = CHECKING

        # ── CORRECTING: small angle fix then drive again ──────────
        elif self.state == CORRECTING:
            if abs(angle_err) > ANGLE_THRESH:
                turn = min(abs(angle_err), 30)
                if angle_err > 0:
                    self.send({'cmd': 'TURN_L', 'angle': round(turn, 1)})
                else:
                    self.send({'cmd': 'TURN_R', 'angle': round(turn, 1)})
            self.state = DRIVING


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