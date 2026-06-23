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
BOT_IPS = {
    1: '192.168.1.7',
    2: '192.168.1.8',
}
UDP_PORT          = 4210
ARRIVE_DIST_LOAD  = 100   # mm
ARRIVE_DIST_DROP  = 150   # mm
ANGLE_THRESH      = 15    # degrees — acceptable heading error
COLLISION_RADIUS  = 300   # mm — 30 cm collision check radius
FILTER_N          = 5     # smaller = faster response
CMD_TIMEOUT       = 8.0   # seconds — max wait for bot done reply
COLLISION_CHECK_HZ = 10   # Hz — how fast collision thread polls


# ─── THREAD-SAFE POSE ────────────────────────────────────────────
class SafePose:
    """Thread-safe position + yaw storage."""
    def __init__(self, n=5):
        self._lock  = threading.Lock()
        self._xs    = deque(maxlen=n)
        self._ys    = deque(maxlen=n)
        self._yaws  = deque(maxlen=n)

    def update(self, x, y, yaw):
        with self._lock:
            self._xs.append(x)
            self._ys.append(y)
            self._yaws.append(yaw)

    def get(self):
        with self._lock:
            if len(self._xs) < 3:
                return None
            x   = sum(self._xs) / len(self._xs)
            y   = sum(self._ys) / len(self._ys)
            sins = sum(math.sin(math.radians(a)) for a in self._yaws)
            coss = sum(math.cos(math.radians(a)) for a in self._yaws)
            yaw  = math.degrees(math.atan2(sins, coss))
            return x, y, yaw

    def ready(self):
        with self._lock:
            return len(self._xs) >= 3

    def clear(self):
        with self._lock:
            self._xs.clear()
            self._ys.clear()
            self._yaws.clear()


# ─── COLLISION MONITOR (separate thread) ─────────────────────────
class CollisionMonitor:
    """
    Runs in its own thread, completely independent of bot threads.
    Checks all bot-pair distances every 100ms.
    When two bots are within COLLISION_RADIUS:
      - Lower ID bot: allowed to move (no change)
      - Higher ID bot: sent STOP command + flagged as held
    When clear again: higher ID bot flag cleared so it can resume.
    """
    def __init__(self, bots, logger):
        self.bots    = bots    # dict {bot_id: BotController}
        self.log     = logger
        self._running = True

        # Which bots are currently held by collision logic
        # {bot_id: True/False}
        self._held = {bid: False for bid in bots}
        self._held_lock = threading.Lock()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info('[CollisionMonitor] started')

    def is_held(self, bot_id):
        """BotController calls this to check if it should pause."""
        with self._held_lock:
            return self._held.get(bot_id, False)

    def stop(self):
        self._running = False

    def _run(self):
        interval = 1.0 / COLLISION_CHECK_HZ
        bot_ids  = sorted(self.bots.keys())

        while self._running:
            time.sleep(interval)

            # Check every pair
            for i in range(len(bot_ids)):
                for j in range(i + 1, len(bot_ids)):
                    id_low  = bot_ids[i]   # lower ID → priority (always moves)
                    id_high = bot_ids[j]   # higher ID → stops on collision

                    pos_low  = self.bots[id_low].pose.get()
                    pos_high = self.bots[id_high].pose.get()

                    if pos_low is None or pos_high is None:
                        continue

                    dist = math.dist(
                        (pos_low[0],  pos_low[1]),
                        (pos_high[0], pos_high[1])
                    )

                    with self._held_lock:
                        currently_held = self._held[id_high]

                    if dist < COLLISION_RADIUS:
                        if not currently_held:
                            # New collision — stop higher ID bot
                            self.log.warn(
                                f'[CollisionMonitor] Bot{id_low} vs Bot{id_high} '
                                f'dist={dist:.0f}mm < {COLLISION_RADIUS}mm — '
                                f'STOPPING Bot{id_high}, Bot{id_low} continues'
                            )
                            with self._held_lock:
                                self._held[id_high] = True
                            # Send STOP command to higher ID bot immediately
                            self.bots[id_high]._send_raw({'cmd': 'STOP'})
                        # else: already held, keep sending STOP to be safe
                        else:
                            self.bots[id_high]._send_raw({'cmd': 'STOP'})

                    else:
                        if currently_held:
                            # Just cleared — release higher ID bot
                            self.log.info(
                                f'[CollisionMonitor] Bot{id_low} vs Bot{id_high} '
                                f'dist={dist:.0f}mm — path clear, releasing Bot{id_high}'
                            )
                            with self._held_lock:
                                self._held[id_high] = False
                            # Bot's own _run loop will automatically resume
                            # because is_held() will return False


# ─── BOT CONTROLLER ──────────────────────────────────────────────
class BotController:
    """
    Runs in its own thread. Independently navigates one bot.
    Communicates with CollisionMonitor via is_held() flag.
    """
    def __init__(self, bot_id, ip, logger, collision_monitor_ref, collision_lock):
        self.bot_id           = bot_id
        self.ip               = ip
        self.log              = logger
        self.collision_monitor = collision_monitor_ref   # set after monitor is created
        self.col_lock         = collision_lock

        self.pose       = SafePose(FILTER_N)
        self._load_pose = SafePose(FILTER_N)
        self._drop_pose = SafePose(FILTER_N)

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self._done_event = threading.Event()

        # Control
        self._phase      = 'IDLE'
        self._phase_lock = threading.Lock()
        self._running    = True

        # Start threads
        self._thread    = threading.Thread(target=self._run,     daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        self._rx_thread.start()

    # ── Public API ────────────────────────────────────────────────
    def assign_load(self, load_id):
        with self._phase_lock:
            self._phase   = 'TO_LOAD'
            self._load_id = load_id
        self._load_pose.clear()
        self.pose.clear()
        self.log.info(f'[Bot{self.bot_id}] assigned Load{load_id}')

    def update_load_pose(self, x, y):
        self._load_pose.update(x, y, 0)

    def update_drop_pose(self, x, y):
        self._drop_pose.update(x, y, 0)

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
            self.log.error(f'[Bot{self.bot_id}] UDP send error: {e}')

    def _send_wait(self, cmd):
        """Send command and block until done reply or timeout.
           Also respects collision hold — pauses immediately if held."""
        # Don't even send if currently held
        self._wait_if_held()

        self._done_event.clear()
        self._send_raw(cmd)
        self.log.info(f'[Bot{self.bot_id}] TX → {cmd}')

        # Wait for done, but wake up periodically to check hold flag
        deadline = time.time() + CMD_TIMEOUT
        while self._running:
            remaining = deadline - time.time()
            if remaining <= 0:
                self.log.warn(f'[Bot{self.bot_id}] timeout waiting for done')
                break

            # Check if we got held mid-command
            if self._is_held():
                self.log.warn(
                    f'[Bot{self.bot_id}] held mid-command, waiting for clear...')
                self._wait_if_held()
                # After released, treat as "done" so we re-plan the move
                break

            # Check done with short timeout slice
            got_done = self._done_event.wait(timeout=0.1)
            if got_done:
                break

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

    # ── Collision hold helpers ────────────────────────────────────
    def _is_held(self):
        """Check if collision monitor has flagged this bot to stop."""
        if self.collision_monitor is None:
            return False
        return self.collision_monitor.is_held(self.bot_id)

    def _wait_if_held(self):
        """Block until collision monitor releases this bot."""
        if self._is_held():
            self.log.warn(
                f'[Bot{self.bot_id}] collision hold — waiting for Bot with '
                f'lower ID to clear...')
            while self._running and self._is_held():
                time.sleep(0.1)
            self.log.info(f'[Bot{self.bot_id}] collision cleared — resuming')

    # ── Navigation core ───────────────────────────────────────────
    def _get_error(self, tx, ty):
        pose = self.pose.get()
        if pose is None:
            return None
        bx, by, yaw  = pose
        dx, dy       = tx - bx, ty - by
        dist         = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(-dy, dx))
        angle_err    = (target_angle - yaw + 180) % 360 - 180
        return dist, angle_err, bx, by, yaw

    def _wait_pose(self, timeout=5.0):
        t = time.time()
        while not self.pose.ready():
            if time.time() - t > timeout:
                return False
            time.sleep(0.1)
        return True

    def _navigate_to(self, get_target_fn, arrive_dist):
        """
        Plan-then-execute navigation with collision-aware waits.
        Before every TURN or FORWARD command, checks hold flag.
        """
        while self._running:
            # Check hold at top of every loop iteration
            self._wait_if_held()

            # Wait for fresh pose
            self.pose.clear()
            if not self._wait_pose():
                self.log.warn(f'[Bot{self.bot_id}] no pose — retrying')
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

            # Arrived?
            if dist < arrive_dist:
                return True

            # ── TURN ─────────────────────────────────────────────
            if abs(angle_err) > ANGLE_THRESH:
                self._wait_if_held()   # check before turning
                cmd = 'TURN_L' if angle_err > 0 else 'TURN_R'
                self._send_wait({'cmd': cmd, 'angle': round(abs(angle_err), 1)})
                time.sleep(0.2)

            # Re-check after turn
            self._wait_if_held()
            self.pose.clear()
            if not self._wait_pose(2.0):
                continue
            target = get_target_fn()
            if target is None:
                continue
            tx, ty = target
            err    = self._get_error(tx, ty)
            if err is None:
                continue
            dist, angle_err, bx, by, yaw = err

            if dist < arrive_dist:
                return True

            # ── DRIVE ─────────────────────────────────────────────
            self._wait_if_held()   # check before driving
            drive_dist = max(dist - arrive_dist + 20, 50)
            self._send_wait({'cmd': 'FORWARD', 'dist': round(drive_dist, 1)})
            time.sleep(0.3)

        return False

    # ── Main bot thread ───────────────────────────────────────────
    def _run(self):
        self.log.info(f'[Bot{self.bot_id}] thread started')

        while self._running:
            phase = self.get_phase()

            if phase == 'IDLE':
                time.sleep(0.1)
                continue

            # ── TO_LOAD ───────────────────────────────────────────
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

            # ── TO_DROP ───────────────────────────────────────────
            elif phase == 'TO_DROP':
                self.log.info(f'[Bot{self.bot_id}] navigating to dropzone')

                self.log.info(f'[Bot{self.bot_id}] waiting for dropzone access')
                self.col_lock.acquire()
                self.log.info(f'[Bot{self.bot_id}] dropzone access granted')

                arrived = self._navigate_to(
                    lambda: self._drop_pose.get()[:2]
                        if self._drop_pose.ready() else None,
                    ARRIVE_DIST_DROP)

                if arrived:
                    self.log.info(f'[Bot{self.bot_id}] RELEASING')
                    self._send_wait({'cmd': 'RELEASE'})
                    self.col_lock.release()
                    self.log.info(f'[Bot{self.bot_id}] dropzone released')

                    self.log.info(f'[Bot{self.bot_id}] backing up')
                    self._send_wait({'cmd': 'BACKWARD', 'dist': 150})
                    with self._phase_lock:
                        self._phase = 'IDLE'
                else:
                    self.col_lock.release()

        self.log.info(f'[Bot{self.bot_id}] thread stopped')


# ─── TASK MANAGER NODE ───────────────────────────────────────────
class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self._dropzone_lock = threading.Lock()

        # ── Create bots first (collision_monitor set to None initially) ──
        self.bots = {
            1: BotController(
                1, BOT_IPS[1], self.get_logger(),
                None,   # collision_monitor injected below
                self._dropzone_lock),
            2: BotController(
                2, BOT_IPS[2], self.get_logger(),
                None,
                self._dropzone_lock),
        }

        # ── Create collision monitor with reference to all bots ──
        self._collision_monitor = CollisionMonitor(
            self.bots, self.get_logger())

        # ── Inject monitor reference into each bot ──
        for bot in self.bots.values():
            bot.collision_monitor = self._collision_monitor

        self.loads      = {}
        self.delivered  = set()
        self.assigned   = {}
        self._data_lock = threading.Lock()

        self.create_subscription(String, '/bot_poses', self.bots_cb,  10)
        self.create_subscription(String, '/loads',     self.loads_cb, 10)
        self.create_subscription(String, '/dropzone',  self.drop_cb,  10)
        self.pub_task = self.create_publisher(String, '/task_info', 10)

        self.create_timer(0.3, self.manage)

        self.get_logger().info(
            f'TaskManager ready — collision radius={COLLISION_RADIUS}mm '
            f'(lower bot ID has priority)')

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
                    for bot_id, assigned_lid in self.assigned.items():
                        if assigned_lid == lid:
                            self.bots[bot_id].update_load_pose(
                                l['x_mm'], l['y_mm'])

    def drop_cb(self, msg):
        d = json.loads(msg.data)
        for bot in self.bots.values():
            bot.update_drop_pose(d['x_mm'], d['y_mm'])

    # ── Assignment — nearest load to nearest idle bot ─────────────
    def manage(self):
        with self._data_lock:
            unassigned = [
                lid for lid in self.loads
                if lid not in self.delivered
                and lid not in self.assigned.values()
            ]

            idle_bots = [
                b for b in self.bots.values()
                if b.is_idle() and b.pose.ready()
            ]

            for bot in idle_bots:
                if not unassigned:
                    break
                pose = bot.pose.get()
                if pose is None:
                    continue
                bx, by = pose[0], pose[1]

                best_lid  = None
                best_dist = float('inf')
                for lid in unassigned:
                    ldata = self.loads[lid]
                    d = math.dist((bx, by), (ldata['x_mm'], ldata['y_mm']))
                    if d < best_dist:
                        best_dist = d
                        best_lid  = lid

                if best_lid is None:
                    continue

                self.assigned[best_lid] = bot.bot_id
                unassigned.remove(best_lid)
                bot.assign_load(best_lid)

            for bot in self.bots.values():
                if bot.is_idle():
                    to_remove = [
                        lid for lid, bid in self.assigned.items()
                        if bid == bot.bot_id and lid in self.delivered
                    ]
                    for lid in to_remove:
                        self.assigned.pop(lid, None)

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
                'loads':      load_list,
                'bot1_phase': self.bots[1].get_phase(),
                'bot2_phase': self.bots[2].get_phase(),
            })))

        with self._data_lock:
            remaining = [
                lid for lid in self.loads if lid not in self.delivered]
        if not remaining and all(b.is_idle() for b in self.bots.values()):
            self.get_logger().info('=== ALL LOADS DELIVERED ===')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._collision_monitor.stop()
        for bot in node.bots.values():
            bot.stop()
        node.destroy_node()
        rclpy.shutdown()