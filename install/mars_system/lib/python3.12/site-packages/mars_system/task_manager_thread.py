import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import socket
import threading
import time
from collections import deque

# ─── CONFIG ──────────────────────────────────────────────────────
BOT_IPS = {1: '10.149.68.145', 2: '10.149.68.167'}
UDP_PORT = 4210

# Bot1 → dropzone 1 (marker 20) → odd loads  (1, 3)
# Bot2 → dropzone 2 (marker 21) → even loads (2, 4)
BOT_DROPZONE = {1: 1, 2: 2}

ARRIVE_DIST_LOAD = 100
ARRIVE_DIST_DROP = 150
ANGLE_THRESH     = 15
SAFE_DIST        = 200
FILTER_N         = 5
CMD_TIMEOUT      = 10.0


# ─── THREAD-SAFE POSE ────────────────────────────────────────────
class SafePose:
    def __init__(self, n=5):
        self._lock = threading.Lock()
        self._xs   = deque(maxlen=n)
        self._ys   = deque(maxlen=n)
        self._yaws = deque(maxlen=n)

    def update(self, x, y, yaw=0):
        with self._lock:
            self._xs.append(x)
            self._ys.append(y)
            self._yaws.append(yaw)

    def get(self):
        with self._lock:
            if len(self._xs) < 3:
                return None
            x    = sum(self._xs) / len(self._xs)
            y    = sum(self._ys) / len(self._ys)
            sins = sum(math.sin(math.radians(a)) for a in self._yaws)
            coss = sum(math.cos(math.radians(a)) for a in self._yaws)
            yaw  = math.degrees(math.atan2(sins, coss))
            return x, y, yaw

    def ready(self):
        with self._lock:
            return len(self._xs) >= 3

    def clear(self):
        with self._lock:
            self._xs.clear(); self._ys.clear(); self._yaws.clear()


# ─── BOT CONTROLLER THREAD ───────────────────────────────────────
class BotController:
    def __init__(self, bot_id, ip, dropzone_num, logger,
                 get_other_pos, get_drop_pose):
        self.bot_id       = bot_id
        self.ip           = ip
        self.dropzone_num = dropzone_num
        self.log          = logger
        self.get_other_pos = get_other_pos  # fn → (x,y) or None
        self.get_drop_pose = get_drop_pose  # fn → (x,y) or None

        self.pose       = SafePose(FILTER_N)
        self._load_pose = SafePose(FILTER_N)

        self.sock         = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self._done_event  = threading.Event()

        self._phase      = 'IDLE'
        self._phase_lock = threading.Lock()
        self._load_id    = None
        self._running    = True

        # Start threads
        threading.Thread(target=self._run,     daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    # ── Public API ────────────────────────────────────────────────
    def assign_load(self, load_id):
        with self._phase_lock:
            self._phase   = 'TO_LOAD'
            self._load_id = load_id
        self._load_pose.clear()
        self.pose.clear()
        self.log.info(f'[Bot{self.bot_id}] assigned Load{load_id} '
                      f'→ DropZone{self.dropzone_num}')

    def update_load_pose(self, x, y):
        self._load_pose.update(x, y, 0)

    def get_phase(self):
        with self._phase_lock:
            return self._phase

    def is_idle(self):
        return self.get_phase() == 'IDLE'

    def stop(self):
        self._running = False
        self._send_raw({'cmd': 'STOP'})

    # ── UDP ───────────────────────────────────────────────────────
    def _send_raw(self, cmd):
        cmd['id'] = self.bot_id
        try:
            self.sock.sendto(json.dumps(cmd).encode(), (self.ip, UDP_PORT))
        except Exception as e:
            self.log.error(f'[Bot{self.bot_id}] send error: {e}')

    def _send_wait(self, cmd):
        self._done_event.clear()
        self._send_raw(cmd)
        self.log.info(f'[Bot{self.bot_id}] TX → {cmd}')
        if not self._done_event.wait(timeout=CMD_TIMEOUT):
            self.log.warn(f'[Bot{self.bot_id}] timeout')

    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(512)
                d = json.loads(data.decode())
                if d.get('type') == 'done':
                    self._done_event.set()
            except socket.timeout:
                pass
            except Exception:
                pass

    # ── Collision ─────────────────────────────────────────────────
    def _other_too_close(self):
        my = self.pose.get()
        if my is None: return False
        other = self.get_other_pos()
        if other is None: return False
        return math.dist((my[0], my[1]), other) < SAFE_DIST

    def _wait_clear(self):
        while self._running and self._other_too_close():
            self.log.warn(f'[Bot{self.bot_id}] collision hold')
            time.sleep(0.3)

    # ── Navigation ────────────────────────────────────────────────
    def _get_error(self, tx, ty):
        pose = self.pose.get()
        if pose is None: return None
        bx, by, yaw  = pose
        dx, dy        = tx - bx, ty - by
        dist          = math.sqrt(dx*dx + dy*dy)
        target_angle  = math.degrees(math.atan2(-dy, dx))
        angle_err     = (target_angle - yaw + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    def _wait_pose(self, timeout=5.0):
        t = time.time()
        while not self.pose.ready():
            if time.time() - t > timeout: return False
            time.sleep(0.1)
        return True

    def _navigate_to(self, get_target_fn, arrive_dist):
        while self._running:
            self.pose.clear()
            if not self._wait_pose():
                self.log.warn(f'[Bot{self.bot_id}] no pose')
                continue

            target = get_target_fn()
            if target is None:
                time.sleep(0.2)
                continue

            tx, ty = target
            err    = self._get_error(tx, ty)
            if err is None:
                time.sleep(0.1)
                continue

            dist, angle_err, bx, by, yaw = err
            self.log.info(
                f'[Bot{self.bot_id}] '
                f'pos=({bx:.0f},{by:.0f}) yaw={yaw:.0f}° '
                f'target=({tx:.0f},{ty:.0f}) '
                f'dist={dist:.0f}mm err={angle_err:.0f}°')

            if dist < arrive_dist:
                return True

            # Collision check
            self._wait_clear()

            # Turn
            if abs(angle_err) > ANGLE_THRESH:
                cmd = 'TURN_L' if angle_err > 0 else 'TURN_R'
                self._send_wait({'cmd': cmd, 'angle': round(abs(angle_err), 1)})
                time.sleep(0.2)

            # Re-check after turn
            self.pose.clear()
            if not self._wait_pose(2.0): continue
            target = get_target_fn()
            if target is None: continue
            tx, ty = target
            err    = self._get_error(tx, ty)
            if err is None: continue
            dist, angle_err, bx, by, yaw = err

            if dist < arrive_dist:
                return True

            # Collision check before drive
            self._wait_clear()

            # Drive
            drive_dist = max(dist - arrive_dist + 20, 50)
            self._send_wait({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            time.sleep(0.3)

        return False

    # ── Main bot thread ───────────────────────────────────────────
    def _run(self):
        self.log.info(f'[Bot{self.bot_id}] thread started '
                      f'→ DropZone{self.dropzone_num}')

        while self._running:
            phase = self.get_phase()

            if phase == 'IDLE':
                time.sleep(0.1)
                continue

            # TO_LOAD
            if phase == 'TO_LOAD':
                self.log.info(f'[Bot{self.bot_id}] navigating to load')
                arrived = self._navigate_to(
                    lambda: self._load_pose.get()[:2]
                        if self._load_pose.ready() else None,
                    ARRIVE_DIST_LOAD)

                if arrived:
                    self.log.info(f'[Bot{self.bot_id}] GRABBING')
                    self._send_wait({'cmd': 'GRAB'})
                    with self._phase_lock:
                        self._phase = 'TO_DROP'

            # TO_DROP
            elif phase == 'TO_DROP':
                self.log.info(
                    f'[Bot{self.bot_id}] navigating to DropZone{self.dropzone_num}')

                arrived = self._navigate_to(
                    lambda: self.get_drop_pose(),
                    ARRIVE_DIST_DROP)

                if arrived:
                    self.log.info(f'[Bot{self.bot_id}] RELEASING')
                    self._send_wait({'cmd': 'RELEASE'})

                    self.log.info(f'[Bot{self.bot_id}] backing up')
                    self._send_wait({'cmd': 'BACKWARD', 'dist': 80})

                    with self._phase_lock:
                        self._phase = 'IDLE'

        self.log.info(f'[Bot{self.bot_id}] thread stopped')


# ─── TASK MANAGER NODE ───────────────────────────────────────────
class TaskManagerThreaded(Node):
    def __init__(self):
        super().__init__('task_manager_threaded')

        # Drop zone poses (shared, thread-safe)
        self._drop_poses = {
            1: SafePose(FILTER_N),
            2: SafePose(FILTER_N),
        }

        self.bots = {
            1: BotController(
                1, BOT_IPS[1],
                dropzone_num=1,
                logger=self.get_logger(),
                get_other_pos=lambda: self._get_bot_pos(2),
                get_drop_pose=lambda: self._get_drop(1)),
            2: BotController(
                2, BOT_IPS[2],
                dropzone_num=2,
                logger=self.get_logger(),
                get_other_pos=lambda: self._get_bot_pos(1),
                get_drop_pose=lambda: self._get_drop(2)),
        }

        self.loads      = {}
        self.delivered  = set()
        self.assigned   = {}
        self._data_lock = threading.Lock()

        self.create_subscription(String, '/bot_poses',  self.bots_cb,  10)
        self.create_subscription(String, '/loads',      self.loads_cb, 10)
        self.create_subscription(String, '/dropzone_one', self.drop1_cb, 10)
        self.create_subscription(String, '/dropzone_two', self.drop2_cb, 10)

        self.create_timer(0.3, self.manage)
        self.get_logger().info('TaskManager Threaded — Bot1+Bot2, dual dropzone')

    def _get_bot_pos(self, bot_id):
        pose = self.bots[bot_id].pose.get()
        if pose is None: return None
        return (pose[0], pose[1])

    def _get_drop(self, zone_num):
        pose = self._drop_poses[zone_num].get()
        if pose is None: return None
        return (pose[0], pose[1])

    # ── Subscribers ───────────────────────────────────────────────
    def bots_cb(self, msg):
        for b in json.loads(msg.data):
            bid = b['id']
            if bid in self.bots:
                self.bots[bid].pose.update(
                    b['x_mm'], b['y_mm'], b['yaw_deg'])

    def loads_cb(self, msg):
        with self._data_lock:
            for l in json.loads(msg.data):
                lid = l['id']
                if lid not in self.delivered:
                    self.loads[lid] = {'x_mm': l['x_mm'], 'y_mm': l['y_mm']}
                    # update load pose for assigned bot
                    for alid, bid in self.assigned.items():
                        if alid == lid:
                            self.bots[bid].update_load_pose(
                                l['x_mm'], l['y_mm'])

    def drop1_cb(self, msg):
        d = json.loads(msg.data)
        self._drop_poses[1].update(d['x_mm'], d['y_mm'], 0)

    def drop2_cb(self, msg):
        d = json.loads(msg.data)
        self._drop_poses[2].update(d['x_mm'], d['y_mm'], 0)

    # ── Assignment ────────────────────────────────────────────────
    def loads_for_bot(self, bot_id):
        result = []
        for lid in self.loads:
            if lid in self.delivered: continue
            actual = lid + 1
            if bot_id == 1 and actual % 2 == 1:
                result.append(lid)
            elif bot_id == 2 and actual % 2 == 0:
                result.append(lid)
        return result

    def manage(self):
        with self._data_lock:
            for bot in self.bots.values():
                if not bot.is_idle() or not bot.pose.ready():
                    continue
                my_loads = [
                    lid for lid in self.loads_for_bot(bot.bot_id)
                    if lid not in self.assigned
                ]
                if not my_loads:
                    continue
                pose = bot.pose.get()
                if pose is None: continue
                bx, by = pose[0], pose[1]
                best = min(my_loads, key=lambda lid: math.dist(
                    (bx,by),
                    (self.loads[lid]['x_mm'], self.loads[lid]['y_mm'])))
                self.assigned[best] = bot.bot_id
                bot.assign_load(best)

            # Clean up delivered from assigned
            done_lids = [lid for lid in self.assigned if lid in self.delivered]
            for lid in done_lids:
                self.assigned.pop(lid, None)

        with self._data_lock:
            remaining = [lid for lid in self.loads if lid not in self.delivered]
        if not remaining and all(b.is_idle() for b in self.bots.values()):
            self.get_logger().info('=== ALL LOADS DELIVERED ===')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerThreaded()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for bot in node.bots.values():
            bot.stop()
        node.destroy_node()
        rclpy.shutdown()