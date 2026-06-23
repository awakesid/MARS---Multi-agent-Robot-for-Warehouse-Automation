import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_ID   = 1
BOT_IP   = '10.149.68.145'
UDP_PORT = 4210

# Bot1 uses dropzone 1 (marker 20)
# Odd loads  (1,3) → dropzone 1
# Even loads (2,4) → dropzone 2
# Single bot handles all loads, alternates drop zones per load parity

ARRIVE_DIST_LOAD = 100
ARRIVE_DIST_DROP = 150
ANGLE_THRESH     = 15
FILTER_N         = 5

WAITING  = 'WAITING'
TURNING  = 'TURNING'
DRIVING  = 'DRIVING'
CHECKING = 'CHECKING'
IDLE     = 'IDLE'
TO_LOAD  = 'TO_LOAD'
TO_DROP  = 'TO_DROP'
BACKING  = 'BACKING'
DONE     = 'DONE'


class PoseFilter:
    def __init__(self, n=5):
        self.xs = deque(maxlen=n)
        self.ys = deque(maxlen=n)

    def update(self, x, y):
        self.xs.append(x); self.ys.append(y)

    def get(self):
        return sum(self.xs)/len(self.xs), sum(self.ys)/len(self.ys)

    def ready(self, min_n=3):
        return len(self.xs) >= min_n

    def clear(self):
        self.xs.clear(); self.ys.clear()


class YawFilter:
    def __init__(self, n=5):
        self.yaws = deque(maxlen=n)

    def update(self, yaw):
        self.yaws.append(yaw)

    def get(self):
        sins = sum(math.sin(math.radians(y)) for y in self.yaws)
        coss = sum(math.cos(math.radians(y)) for y in self.yaws)
        return math.degrees(math.atan2(sins, coss))

    def ready(self, min_n=3):
        return len(self.yaws) >= min_n

    def clear(self):
        self.yaws.clear()


class TaskManagerSingle(Node):
    def __init__(self):
        super().__init__('task_manager_single')

        self.bot_filter   = PoseFilter(FILTER_N)
        self.bot_yaw      = YawFilter(FILTER_N)
        self.load_filter  = PoseFilter(FILTER_N)
        self.drop1_filter = PoseFilter(FILTER_N)  # marker 20
        self.drop2_filter = PoseFilter(FILTER_N)  # marker 21

        self.loads           = {}
        self.delivered       = set()
        self.current_load_id = None

        self.phase        = IDLE
        self.state        = WAITING
        self.busy         = False
        self.settle_ticks = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)

        self.create_subscription(String, '/bot_poses',  self.bots_cb,  10)
        self.create_subscription(String, '/loads',      self.loads_cb, 10)
        self.create_subscription(String, '/dropzone_one', self.drop1_cb, 10)
        self.create_subscription(String, '/dropzone_two', self.drop2_cb, 10)

        self.create_timer(0.2, self.loop)
        self.get_logger().info('TaskManager Single Bot — dual dropzone')

    # ── Subscribers ───────────────────────────────────────────────
    def bots_cb(self, msg):
        for b in json.loads(msg.data):
            if b['id'] == BOT_ID:
                self.bot_filter.update(b['x_mm'], b['y_mm'])
                self.bot_yaw.update(b['yaw_deg'])

    def loads_cb(self, msg):
        for l in json.loads(msg.data):
            lid = l['id']
            if lid not in self.delivered:
                self.loads[lid] = {'x_mm': l['x_mm'], 'y_mm': l['y_mm']}
                if lid == self.current_load_id:
                    self.load_filter.update(l['x_mm'], l['y_mm'])

    def drop1_cb(self, msg):
        d = json.loads(msg.data)
        self.drop1_filter.update(d['x_mm'], d['y_mm'])

    def drop2_cb(self, msg):
        d = json.loads(msg.data)
        self.drop2_filter.update(d['x_mm'], d['y_mm'])

    # ── Helpers ───────────────────────────────────────────────────
    def drop_filter_for(self, load_id):
        """Odd loads → dropzone1, even loads → dropzone2."""
        actual_load_num = load_id + 1  # id 0=load1, 1=load2, 2=load3, 3=load4
        if actual_load_num % 2 == 1:
            return self.drop1_filter
        else:
            return self.drop2_filter

    def drop_name_for(self, load_id):
        actual_load_num = load_id + 1
        return 'DropZone1' if actual_load_num % 2 == 1 else 'DropZone2'

    def arrive_dist(self):
        return ARRIVE_DIST_LOAD if self.phase == TO_LOAD else ARRIVE_DIST_DROP

    def nearest_load(self):
        if not self.bot_filter.ready():
            return None, None
        bx, by    = self.bot_filter.get()
        best_id   = None
        best_dist = float('inf')
        for lid, ldata in self.loads.items():
            if lid in self.delivered:
                continue
            d = math.dist((bx, by), (ldata['x_mm'], ldata['y_mm']))
            if d < best_dist:
                best_dist = d
                best_id   = lid
        return best_id, best_dist

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
                self.get_logger().info('RX ← done')
        except socket.timeout:
            pass

    def get_error(self, tx, ty):
        bx, by       = self.bot_filter.get()
        yaw          = self.bot_yaw.get()
        dx, dy       = tx - bx, ty - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = (target_angle - yaw + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    def reset_filters(self):
        self.bot_filter.clear()
        self.bot_yaw.clear()

    # ── Main loop ─────────────────────────────────────────────────
    def loop(self):
        self.recv()

        if self.phase == DONE:
            return

        # BACKING
        if self.phase == BACKING:
            if self.busy:
                return
            self.send({'cmd': 'BACKWARD', 'dist': 150})
            self.phase        = IDLE
            self.state        = WAITING
            self.settle_ticks = 0
            self.reset_filters()
            return

        # IDLE: pick nearest load
        if self.phase == IDLE:
            if not self.bot_filter.ready() or not self.loads:
                self.get_logger().warn('Waiting for bot + loads...')
                return
            lid, dist = self.nearest_load()
            if lid is None:
                self.get_logger().info('All loads delivered!')
                self.phase = DONE
                return
            self.current_load_id = lid
            self.load_filter.clear()
            self.phase = TO_LOAD
            self.state = WAITING
            self.reset_filters()
            self.get_logger().info(
                f'Assigned Load{lid} → {self.drop_name_for(lid)} '
                f'dist={dist:.0f}mm')
            return

        # Get target
        if self.phase == TO_LOAD:
            if not self.load_filter.ready():
                self.get_logger().warn(
                    f'Waiting for Load{self.current_load_id}...')
                return
            tx, ty = self.load_filter.get()
        else:  # TO_DROP
            df = self.drop_filter_for(self.current_load_id)
            if not df.ready():
                self.get_logger().warn(
                    f'Waiting for {self.drop_name_for(self.current_load_id)}...')
                return
            tx, ty = df.get()

        if not self.bot_filter.ready() or not self.bot_yaw.ready():
            return
        if self.busy:
            return

        dist, angle_err, bx, by, yaw = self.get_error(tx, ty)

        self.get_logger().info(
            f'[{self.phase}][{self.state}] '
            f'bot=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
            f'target=({tx:.0f},{ty:.0f}) '
            f'dist={dist:.0f}mm err={angle_err:.0f}°')

        # CHECKING
        if self.state == CHECKING:
            self.settle_ticks += 1
            if self.settle_ticks < 3:
                return
            self.reset_filters()
            self.state = WAITING
            return

        # WAITING
        if self.state == WAITING:
            if not self.bot_filter.ready(min_n=FILTER_N):
                return
            self.state = TURNING
            return

        # ARRIVED
        if dist < self.arrive_dist():
            if self.phase == TO_LOAD:
                self.get_logger().info(
                    f'Load{self.current_load_id} — GRABBING → '
                    f'{self.drop_name_for(self.current_load_id)}')
                self.send({'cmd': 'GRAB'})
                self.delivered.add(self.current_load_id)
                self.loads.pop(self.current_load_id, None)
                self.phase = TO_DROP
            else:
                self.get_logger().info('RELEASING → backing up')
                self.send({'cmd': 'RELEASE'})
                self.phase = BACKING
            self.state        = WAITING
            self.settle_ticks = 0
            self.reset_filters()
            return

        # TURNING
        if self.state == TURNING:
            if abs(angle_err) > ANGLE_THRESH:
                cmd = 'TURN_L' if angle_err > 0 else 'TURN_R'
                self.send({'cmd': cmd, 'angle': round(abs(angle_err), 1)})
                return
            self.state = DRIVING
            return

        # DRIVING
        if self.state == DRIVING:
            drive_dist = max(dist - self.arrive_dist() + 20, 50)
            self.send({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            self.settle_ticks = 0
            self.state = CHECKING


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerSingle()
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