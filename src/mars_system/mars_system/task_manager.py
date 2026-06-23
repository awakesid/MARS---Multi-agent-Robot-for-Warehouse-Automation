import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_IPS = {
    1: '192.168.1.17',
    2: '192.168.1.30',
}
UDP_PORT = 4210

ARRIVE_DIST_LOAD = 120    # mm
ARRIVE_DIST_DROP = 160  # mm
ANGLE_THRESH     = 15    # degrees
FILTER_N         = 8
SAFE_DIST        = 300   # mm — min distance between bots before waiting

# ─── STATES / PHASES ─────────────────────────────────────────────
WAITING  = 'WAITING'
TURNING  = 'TURNING'
DRIVING  = 'DRIVING'
CHECKING = 'CHECKING'
IDLE     = 'IDLE'
TO_LOAD  = 'TO_LOAD'
TO_DROP  = 'TO_DROP'
BACKING  = 'BACKING'
WAITING_CLEAR = 'WAITING_CLEAR'  # collision avoidance wait
DONE     = 'DONE'


# ─── FILTERS ─────────────────────────────────────────────────────
class PoseFilter:
    def __init__(self, n=8):
        self.xs = deque(maxlen=n)
        self.ys = deque(maxlen=n)

    def update(self, x, y):
        self.xs.append(x); self.ys.append(y)

    def get(self):
        return sum(self.xs)/len(self.xs), sum(self.ys)/len(self.ys)

    def ready(self, min_n=5):
        return len(self.xs) >= min_n

    def clear(self):
        self.xs.clear(); self.ys.clear()


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


# ─── PER-BOT STATE ───────────────────────────────────────────────
class BotState:
    def __init__(self, bot_id, ip):
        self.bot_id  = bot_id
        self.ip      = ip
        self.priority = bot_id  # Bot1 has higher priority (lower number)

        self.bot_filter  = PoseFilter(FILTER_N)
        self.bot_yaw     = YawFilter(FILTER_N)
        self.load_filter = PoseFilter(FILTER_N)

        self.phase           = IDLE
        self.state           = WAITING
        self.busy            = False
        self.settle_ticks    = 0
        self.current_load_id = None

        # UDP socket per bot
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)

    def send(self, cmd: dict, logger):
        cmd['id'] = self.bot_id
        self.sock.sendto(json.dumps(cmd).encode(), (self.ip, UDP_PORT))
        self.busy = True
        logger.info(f'[Bot{self.bot_id}] TX → {cmd}')

    def recv(self, logger):
        try:
            data, _ = self.sock.recvfrom(512)
            d = json.loads(data.decode())
            if d.get('type') == 'done':
                self.busy = False
                logger.info(f'[Bot{self.bot_id}] RX ← done')
        except socket.timeout:
            pass

    def arrive_dist(self):
        return ARRIVE_DIST_LOAD if self.phase == TO_LOAD else ARRIVE_DIST_DROP

    def get_pos(self):
        if self.bot_filter.ready():
            return self.bot_filter.get()
        return None

    def reset_filters(self):
        self.bot_filter.clear()
        self.bot_yaw.clear()


# ─── TASK MANAGER ────────────────────────────────────────────────
class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.bots = {
            1: BotState(1, BOT_IPS[1]),
            2: BotState(2, BOT_IPS[2]),
        }

        self.drop_filter = PoseFilter(FILTER_N)
        self.loads       = {}    # {id: {x_mm, y_mm}}
        self.delivered   = set()
        self.assigned    = {}    # {load_id: bot_id}

        self.create_subscription(String, '/bot_poses', self.bots_cb,  10)
        self.create_subscription(String, '/loads',     self.loads_cb, 10)
        self.create_subscription(String, '/dropzone',  self.drop_cb,  10)
        self.pub_task = self.create_publisher(String, '/task_info', 10)

        self.create_timer(0.2, self.loop)
        self.get_logger().info('TaskManager ready — Bot1 & Bot2, nearest load first')

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
                # update load filter for assigned bot
                for bot in self.bots.values():
                    if bot.current_load_id == lid:
                        bot.load_filter.update(l['x_mm'], l['y_mm'])

    def drop_cb(self, msg):
        d = json.loads(msg.data)
        self.drop_filter.update(d['x_mm'], d['y_mm'])

    # ── Collision check ───────────────────────────────────────────
    def too_close(self, bot_id):
        """Returns True if this bot is too close to a higher-priority bot."""
        my_pos = self.bots[bot_id].get_pos()
        if my_pos is None:
            return False
        for other_id, other in self.bots.items():
            if other_id == bot_id:
                continue
            if other.priority >= self.bots[bot_id].priority:
                continue  # only yield to higher priority (lower id)
            other_pos = other.get_pos()
            if other_pos is None:
                continue
            dist = math.dist(my_pos, other_pos)
            if dist < SAFE_DIST:
                return True
        return False

    # ── Assign loads to idle bots ─────────────────────────────────
    def assign_loads(self):
        unassigned_loads = [
            lid for lid in self.loads
            if lid not in self.delivered and lid not in self.assigned
        ]
        idle_bots = [
            b for b in self.bots.values()
            if b.phase == IDLE and b.bot_filter.ready()
        ]

        for bot in idle_bots:
            if not unassigned_loads:
                break
            bx, by = bot.bot_filter.get()
            # find nearest unassigned load
            best_lid  = None
            best_dist = float('inf')
            for lid in unassigned_loads:
                ldata = self.loads[lid]
                d = math.dist((bx, by), (ldata['x_mm'], ldata['y_mm']))
                if d < best_dist:
                    best_dist = d
                    best_lid  = lid
            if best_lid is None:
                continue

            # assign
            bot.current_load_id = best_lid
            bot.load_filter.clear()
            bot.phase = TO_LOAD
            bot.state = WAITING
            bot.reset_filters()
            self.assigned[best_lid] = bot.bot_id
            unassigned_loads.remove(best_lid)
            self.get_logger().info(
                f'[Bot{bot.bot_id}] assigned Load{best_lid} '
                f'at {best_dist:.0f}mm')

    # ── Publish viz info ──────────────────────────────────────────
    def publish_task_info(self):
        load_list = []
        for lid, ldata in self.loads.items():
            if lid in self.delivered:
                continue
            load_list.append({
                'id':       lid,
                'x_mm':     ldata['x_mm'],
                'y_mm':     ldata['y_mm'],
                'assigned': self.assigned.get(lid),
            })
        self.pub_task.publish(String(data=json.dumps({
            'loads': load_list,
            'bot1_phase': self.bots[1].phase,
            'bot2_phase': self.bots[2].phase,
        })))

    # ── Error to target ───────────────────────────────────────────
    def get_error(self, bot, tx, ty):
        bx, by       = bot.bot_filter.get()
        yaw          = bot.bot_yaw.get()
        dx, dy       = tx - bx, ty - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = (target_angle - yaw + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    # ── Control single bot ────────────────────────────────────────
    def control_bot(self, bot: BotState):
        bot.recv(self.get_logger())

        if bot.phase == DONE:
            return

        # BACKING
        if bot.phase == BACKING:
            if bot.busy:
                return
            self.get_logger().info(f'[Bot{bot.bot_id}] Backing up 150mm')
            bot.send({'cmd': 'BACKWARD', 'dist': 150}, self.get_logger())
            bot.phase        = IDLE
            bot.state        = WAITING
            bot.settle_ticks = 0
            bot.reset_filters()
            return

        # IDLE — assignment handled separately
        if bot.phase == IDLE:
            return

        # Get target
        if bot.phase == TO_LOAD:
            if not bot.load_filter.ready():
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Waiting for Load{bot.current_load_id}...')
                return
            tx, ty = bot.load_filter.get()
        else:  # TO_DROP
            if not self.drop_filter.ready():
                self.get_logger().warn(f'[Bot{bot.bot_id}] Waiting for dropzone...')
                return
            tx, ty = self.drop_filter.get()

        if not bot.bot_filter.ready() or not bot.bot_yaw.ready():
            return
        if bot.busy:
            return

        # ── Collision avoidance ───────────────────────────────────
        if bot.state == WAITING_CLEAR:
            if self.too_close(bot.bot_id):
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Too close — waiting...')
                return
            else:
                self.get_logger().info(f'[Bot{bot.bot_id}] Path clear — resuming')
                bot.state = TURNING
                return

        dist, angle_err, bx, by, yaw = self.get_error(bot, tx, ty)

        self.get_logger().info(
            f'[Bot{bot.bot_id}][{bot.phase}][{bot.state}] '
            f'bot=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
            f'target=({tx:.0f},{ty:.0f}) '
            f'dist={dist:.0f}mm err={angle_err:.0f}°')

        # CHECKING
        if bot.state == CHECKING:
            bot.settle_ticks += 1
            if bot.settle_ticks < 5:
                return
            bot.reset_filters()
            bot.state = WAITING
            return

        # WAITING
        if bot.state == WAITING:
            if not bot.bot_filter.ready(min_n=FILTER_N):
                return
            # Check collision before planning
            if self.too_close(bot.bot_id):
                bot.state = WAITING_CLEAR
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Too close to other bot — holding')
                return
            bot.state = TURNING
            return

        # ARRIVED
        if dist < bot.arrive_dist():
            if bot.phase == TO_LOAD:
                self.get_logger().info(
                    f'[Bot{bot.bot_id}] Load{bot.current_load_id} — GRABBING')
                bot.send({'cmd': 'GRAB'}, self.get_logger())
                self.delivered.add(bot.current_load_id)
                self.loads.pop(bot.current_load_id, None)
                self.assigned.pop(bot.current_load_id, None)
                bot.phase = TO_DROP
            else:
                self.get_logger().info(f'[Bot{bot.bot_id}] At dropzone — RELEASING')
                bot.send({'cmd': 'RELEASE'}, self.get_logger())
                bot.phase = BACKING
            bot.state        = WAITING
            bot.settle_ticks = 0
            bot.reset_filters()
            return

        # TURNING
        if bot.state == TURNING:
            if abs(angle_err) > ANGLE_THRESH:
                cmd = 'TURN_L' if angle_err > 0 else 'TURN_R'
                bot.send({'cmd': cmd, 'angle': round(abs(angle_err), 1)},
                         self.get_logger())
                return
            bot.state = DRIVING
            return

        # DRIVING
        if bot.state == DRIVING:
            # Check collision before driving
            if self.too_close(bot.bot_id):
                bot.state = WAITING_CLEAR
                self.get_logger().warn(
                    f'[Bot{bot.bot_id}] Too close — stopping before drive')
                bot.send({'cmd': 'STOP'}, self.get_logger())
                bot.busy = False
                return
            arrive = bot.arrive_dist()
            drive_dist = max(dist - arrive + 20, 50)
            bot.send({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)},
                     self.get_logger())
            bot.settle_ticks = 0
            bot.state = CHECKING

    # ── Main loop ─────────────────────────────────────────────────
    def loop(self):
        self.publish_task_info()

        # Assign loads to idle bots
        self.assign_loads()

        # Check if all done
        remaining = [
            lid for lid in self.loads
            if lid not in self.delivered
        ]
        if not remaining:
            all_idle = all(b.phase in [IDLE, DONE] for b in self.bots.values())
            if all_idle and self.loads == {}:
                self.get_logger().info('=== ALL LOADS DELIVERED — DONE ===')
                for bot in self.bots.values():
                    bot.phase = DONE
                return

        # Control each bot
        for bot in self.bots.values():
            self.control_bot(bot)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for bot in node.bots.values():
            bot.sock.sendto(
                json.dumps({'cmd': 'STOP', 'id': bot.bot_id}).encode(),
                (bot.ip, UDP_PORT))
        node.destroy_node()
        rclpy.shutdown()