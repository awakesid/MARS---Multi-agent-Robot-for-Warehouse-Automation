import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_IPS = {1: '192.168.1.5', 2: '192.168.1.7'}
UDP_PORT = 4210

# Bot1 → dropzone 1 (marker 20) → odd loads  (1, 3)
# Bot2 → dropzone 2 (marker 21) → even loads (2, 4)
BOT_DROPZONE = {1: 1, 2: 2}

ARRIVE_DIST_LOAD = 100
ARRIVE_DIST_DROP = 150
ANGLE_THRESH     = 15
SAFE_DIST        = 200
FILTER_N         = 5

WAITING  = 'WAITING'
TURNING  = 'TURNING'
DRIVING  = 'DRIVING'
CHECKING = 'CHECKING'
IDLE     = 'IDLE'
TO_LOAD  = 'TO_LOAD'
TO_DROP  = 'TO_DROP'
BACKING  = 'BACKING'
HOLDING  = 'HOLDING'
DONE     = 'DONE'


class PoseFilter:
    def __init__(self, n=5):
        self.xs = deque(maxlen=n); self.ys = deque(maxlen=n)

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


class BotData:
    def __init__(self, bot_id):
        self.bot_id      = bot_id
        self.bot_filter  = PoseFilter(FILTER_N)
        self.bot_yaw     = YawFilter(FILTER_N)
        self.load_filter = PoseFilter(FILTER_N)

        self.phase           = IDLE
        self.state           = WAITING
        self.busy            = False
        self.settle_ticks    = 0
        self.current_load_id = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)

    def get_pos(self):
        if self.bot_filter.ready():
            return self.bot_filter.get()
        return None

    def arrive_dist(self):
        return ARRIVE_DIST_LOAD if self.phase == TO_LOAD else ARRIVE_DIST_DROP

    def reset_filters(self):
        self.bot_filter.clear(); self.bot_yaw.clear()


class TaskManagerPriority(Node):
    def __init__(self):
        super().__init__('task_manager_priority')

        self.bots = {1: BotData(1), 2: BotData(2)}

        self.drop1_filter = PoseFilter(FILTER_N)
        self.drop2_filter = PoseFilter(FILTER_N)

        self.loads     = {}
        self.delivered = set()
        self.assigned  = {}   # {load_id: bot_id}

        self.create_subscription(String, '/bot_poses',    self.bots_cb,  10)
        self.create_subscription(String, '/loads',        self.loads_cb, 10)
        self.create_subscription(String, '/dropzone_one', self.drop1_cb, 10)
        self.create_subscription(String, '/dropzone_two', self.drop2_cb, 10)

        self.create_timer(0.2, self.loop)
        self.get_logger().info('TaskManager Priority — Bot1+Bot2, dual dropzone')

    # ── Subscribers ───────────────────────────────────────────────
    def bots_cb(self, msg):
        for b in json.loads(msg.data):
            bid = b['id']
            if bid in self.bots:
                self.bots[bid].bot_filter.update(b['x_mm'], b['y_mm'])
                self.bots[bid].bot_yaw.update(b['yaw_deg'])

    def loads_cb(self, msg):
        for l in json.loads(msg.data):
            lid = l['id']
            if lid not in self.delivered:
                self.loads[lid] = {'x_mm': l['x_mm'], 'y_mm': l['y_mm']}
                # FIX: assigned is {load_id: bot_id} — look up by lid directly
                if lid in self.assigned:
                    bot_id = self.assigned[lid]
                    self.bots[bot_id].load_filter.update(l['x_mm'], l['y_mm'])

    def drop1_cb(self, msg):
        d = json.loads(msg.data)
        self.drop1_filter.update(d['x_mm'], d['y_mm'])

    def drop2_cb(self, msg):
        d = json.loads(msg.data)
        self.drop2_filter.update(d['x_mm'], d['y_mm'])

    # ── Helpers ───────────────────────────────────────────────────
    def drop_filter_for_bot(self, bot_id):
        return self.drop1_filter if BOT_DROPZONE[bot_id] == 1 \
               else self.drop2_filter

    def loads_for_bot(self, bot_id):
        """Bot1 takes odd loads, Bot2 takes even loads."""
        result = []
        for lid in self.loads:
            if lid in self.delivered:
                continue
            actual = lid + 1  # id 0=load1, etc
            if bot_id == 1 and actual % 2 == 1:
                result.append(lid)
            elif bot_id == 2 and actual % 2 == 0:
                result.append(lid)
        return result

    def too_close(self, bot_id):
        """Lower priority bot (Bot2) yields to Bot1."""
        if bot_id == 1:
            return False   # Bot1 never yields
        p1 = self.bots[1].get_pos()
        p2 = self.bots[2].get_pos()
        if p1 is None or p2 is None:
            return False
        return math.dist(p1, p2) < SAFE_DIST

    def send(self, bot: BotData, cmd):
        cmd['id'] = bot.bot_id
        bot.sock.sendto(
            json.dumps(cmd).encode(), (BOT_IPS[bot.bot_id], UDP_PORT))
        bot.busy = True
        self.get_logger().info(f'[Bot{bot.bot_id}] TX → {cmd}')

    def recv_all(self):
        for bot in self.bots.values():
            try:
                data, _ = bot.sock.recvfrom(512)
                d = json.loads(data.decode())
                if d.get('type') == 'done':
                    bot.busy = False
                    self.get_logger().info(f'[Bot{bot.bot_id}] RX ← done')
            except socket.timeout:
                pass

    def get_error(self, bot: BotData, tx, ty):
        bx, by       = bot.bot_filter.get()
        yaw          = bot.bot_yaw.get()
        dx, dy       = tx - bx, ty - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = (target_angle - yaw + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    # ── Assign loads ──────────────────────────────────────────────
    def assign_loads(self):
        for bot in self.bots.values():
            if bot.phase != IDLE:
                continue
            if not bot.bot_filter.ready():
                continue
            my_loads = [
                lid for lid in self.loads_for_bot(bot.bot_id)
                if lid not in self.assigned
            ]
            if not my_loads:
                continue
            bx, by = bot.bot_filter.get()
            best_lid = min(my_loads, key=lambda lid: math.dist(
                (bx, by), (self.loads[lid]['x_mm'], self.loads[lid]['y_mm'])))
            best_dist = math.dist(
                (bx, by),
                (self.loads[best_lid]['x_mm'], self.loads[best_lid]['y_mm']))

            # FIX: assigned is {load_id: bot_id}
            self.assigned[best_lid] = bot.bot_id
            bot.current_load_id = best_lid
            bot.load_filter.clear()
            bot.phase = TO_LOAD
            bot.state = WAITING
            bot.reset_filters()
            self.get_logger().info(
                f'[Bot{bot.bot_id}] Load{best_lid} '
                f'→ DropZone{BOT_DROPZONE[bot.bot_id]} '
                f'dist={best_dist:.0f}mm')

    # ── Control single bot ────────────────────────────────────────
    def control_bot(self, bot: BotData):
        if bot.phase == DONE:
            return

        # BACKING
        if bot.phase == BACKING:
            if bot.busy:
                return
            self.send(bot, {'cmd': 'BACKWARD', 'dist': 150})
            bot.phase = IDLE
            bot.state = WAITING
            bot.settle_ticks = 0
            bot.reset_filters()
            # FIX: assigned is {load_id: bot_id}
            to_rm = [lid for lid, bid in self.assigned.items()
                     if bid == bot.bot_id and lid in self.delivered]
            for lid in to_rm:
                self.assigned.pop(lid, None)
            return

        if bot.phase == IDLE:
            return

        # Get target
        if bot.phase == TO_LOAD:
            if not bot.load_filter.ready():
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Waiting for Load{bot.current_load_id}')
                return
            tx, ty = bot.load_filter.get()
        else:
            df = self.drop_filter_for_bot(bot.bot_id)
            if not df.ready():
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Waiting for dropzone')
                return
            tx, ty = df.get()

        if not bot.bot_filter.ready() or not bot.bot_yaw.ready():
            return
        if bot.busy:
            return

        dist, angle_err, bx, by, yaw = self.get_error(bot, tx, ty)

        self.get_logger().info(
            f'[Bot{bot.bot_id}][{bot.phase}][{bot.state}] '
            f'pos=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
            f'target=({tx:.0f},{ty:.0f}) '
            f'dist={dist:.0f}mm err={angle_err:.0f}°')

        # CHECKING — wait for filters to settle after a drive
        if bot.state == CHECKING:
            bot.settle_ticks += 1
            if bot.settle_ticks < 3:
                return
            bot.reset_filters()
            bot.state = WAITING
            return

        # WAITING — wait for fresh filtered pose
        if bot.state == WAITING:
            if not bot.bot_filter.ready(min_n=FILTER_N):
                return
            bot.state = TURNING
            return

        # HOLDING — Bot2 yielding to Bot1
        if bot.state == HOLDING:
            if self.too_close(bot.bot_id):
                self.get_logger().warn(f'[Bot{bot.bot_id}] holding...')
                return
            bot.state = TURNING
            return

        # ARRIVED
        if dist < bot.arrive_dist():
            if bot.phase == TO_LOAD:
                self.get_logger().info(
                    f'[Bot{bot.bot_id}] GRABBING Load{bot.current_load_id}')
                self.send(bot, {'cmd': 'GRAB'})
                self.delivered.add(bot.current_load_id)
                self.loads.pop(bot.current_load_id, None)
                bot.phase = TO_DROP
            else:
                self.get_logger().info(f'[Bot{bot.bot_id}] RELEASING')
                self.send(bot, {'cmd': 'RELEASE'})
                bot.phase = BACKING
            bot.state = WAITING
            bot.settle_ticks = 0
            bot.reset_filters()
            return

        # TURNING
        if bot.state == TURNING:
            if self.too_close(bot.bot_id):
                bot.state = HOLDING
                return
            if abs(angle_err) > ANGLE_THRESH:
                cmd = 'TURN_L' if angle_err > 0 else 'TURN_R'
                self.send(bot, {'cmd': cmd, 'angle': round(abs(angle_err), 1)})
                return
            bot.state = DRIVING
            return

        # DRIVING
        if bot.state == DRIVING:
            if self.too_close(bot.bot_id):
                bot.state = HOLDING
                self.send(bot, {'cmd': 'STOP'})
                bot.busy = False
                return
            drive_dist = max(dist - bot.arrive_dist() + 20, 50)
            self.send(bot, {'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            bot.settle_ticks = 0
            bot.state = CHECKING

    # ── Main loop ─────────────────────────────────────────────────
    def loop(self):
        self.recv_all()
        self.assign_loads()
        # Bot1 first (higher priority)
        self.control_bot(self.bots[1])
        self.control_bot(self.bots[2])

        remaining = [lid for lid in self.loads if lid not in self.delivered]
        if not remaining and all(b.phase in [IDLE, DONE] for b in self.bots.values()):
            self.get_logger().info('=== ALL LOADS DELIVERED ===')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerPriority()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for bid, bot in node.bots.items():
            bot.sock.sendto(
                json.dumps({'cmd': 'STOP', 'id': bid}).encode(),
                (BOT_IPS[bid], UDP_PORT))
        node.destroy_node()
        rclpy.shutdown()