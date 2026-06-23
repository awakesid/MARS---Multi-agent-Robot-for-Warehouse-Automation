import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import json

# ─── MARKER ID MAP ───────────────────────────────────────────────
CORNER_IDS   = {0:'TL', 1:'TR', 2:'BR', 3:'BL'}
BOT_IDS      = {10:1, 11:2, 12:3}
DROPZONE_IDS = {20:1, 21:2}   # marker_id: dropzone_number
LOAD_IDS     = {30, 31, 32, 33}

ARENA_W_MM   = 910
ARENA_H_MM   = 930
DROP_HALF_MM = 150   # 300x300mm drop zone


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge     = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector   = aruco.ArucoDetector(
            self.dictionary, aruco.DetectorParameters())
        self.H = None

        self.sub       = self.create_subscription(
            Image, '/image_raw', self.cb, 10)

        self.pub_bots   = self.create_publisher(String, '/bot_poses',    10)
        self.pub_loads  = self.create_publisher(String, '/loads',        10)
        self.pub_drop1  = self.create_publisher(String, '/dropzone_one',   10)
        self.pub_drop2  = self.create_publisher(String, '/dropzone_two',   10)
        self.pub_ready  = self.create_publisher(String, '/arena_ready',  10)
        self.pub_debug  = self.create_publisher(Image,  '/debug_image',  10)

        # ── Recording ─────────────────────────────────────────────
        RECORD = False
        self.record = RECORD
        if self.record:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.writer = cv2.VideoWriter(
                '/root/mars_ws/recording.avi', fourcc, 15, (640, 480))
            self.get_logger().info('Recording ON → /root/mars_ws/recording.avi')
        else:
            self.writer = None
            self.get_logger().info('Recording OFF')

        cv2.namedWindow('MARS ArUco', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('MARS ArUco', 800, 600)
        self.get_logger().info('ArUco detector ready — dual dropzone mode')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            self.pub_ready.publish(String(data='false'))
            cv2.putText(frame, 'No markers detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            self._show(frame)
            return

        ids_flat = ids.flatten().tolist()
        aruco.drawDetectedMarkers(frame, corners, ids)

        # ── Homography ────────────────────────────────────────────
        cpts = {}
        for i, mid in enumerate(ids_flat):
            if mid in CORNER_IDS:
                c = corners[i][0]
                cpts[mid] = (float(np.mean(c[:,0])), float(np.mean(c[:,1])))

        if len(cpts) == 4:
            src = np.float32([cpts[0], cpts[1], cpts[2], cpts[3]])
            dst = np.float32([
                [0,          0         ],
                [ARENA_W_MM, 0         ],
                [ARENA_W_MM, ARENA_H_MM],
                [0,          ARENA_H_MM],
            ])
            self.H, _ = cv2.findHomography(src, dst)
            self.pub_ready.publish(String(data='true'))
            cv2.putText(frame, 'ARENA LOCKED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            pts = np.float32([cpts[0], cpts[1], cpts[2], cpts[3]])
            cv2.polylines(frame,
                          [pts.reshape((-1,1,2)).astype(np.int32)],
                          isClosed=True, color=(0,255,0), thickness=2)
        else:
            self.pub_ready.publish(String(data='false'))
            for mid, label in CORNER_IDS.items():
                color = (0,255,0) if mid in cpts else (0,0,255)
                text  = f'{label}(ID:{mid}) OK' if mid in cpts \
                        else f'{label}(ID:{mid}) MISSING'
                y = 30 + list(CORNER_IDS.keys()).index(mid) * 25
                cv2.putText(frame, text, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        if self.H is None:
            self._show(frame)
            return

        H_inv = np.linalg.inv(self.H)

        def mm_to_px(mx, my):
            pt  = np.array([[[mx, my]]], dtype=np.float32)
            out = cv2.perspectiveTransform(pt, H_inv)
            return (int(out[0][0][0]), int(out[0][0][1]))

        # ── Bots ─────────────────────────────────────────────────
        bots = []
        for i, mid in enumerate(ids_flat):
            if mid in BOT_IDS:
                c      = corners[i][0]
                x, y   = self._to_mm(np.mean(c[:,0]), np.mean(c[:,1]))
                yaw    = self._yaw(c)
                bot_id = BOT_IDS[mid]
                bots.append({
                    'id':      bot_id,
                    'x_mm':    round(x, 1),
                    'y_mm':    round(y, 1),
                    'yaw_deg': round(yaw, 1),
                })
                cx, cy = int(np.mean(c[:,0])), int(np.mean(c[:,1]))
                cv2.putText(frame,
                    f'Bot{bot_id} ({x:.0f},{y:.0f}) {yaw:.0f}d',
                    (cx-30, cy-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,0), 2)
                ax = int(cx + 30*np.cos(np.radians(yaw)))
                ay = int(cy + 30*np.sin(np.radians(yaw)))
                cv2.arrowedLine(frame, (cx,cy), (ax,ay), (255,100,0), 2)
        if bots:
            self.pub_bots.publish(String(data=json.dumps(bots)))

        # ── Loads ─────────────────────────────────────────────────
        loads = []
        for i, mid in enumerate(ids_flat):
            if mid in LOAD_IDS:
                c    = corners[i][0]
                x, y = self._to_mm(np.mean(c[:,0]), np.mean(c[:,1]))
                loads.append({
                    'id':   mid - 30,
                    'x_mm': round(x, 1),
                    'y_mm': round(y, 1),
                })
                cx, cy = int(np.mean(c[:,0])), int(np.mean(c[:,1]))
                cv2.putText(frame,
                    f'Load{mid-30} ({x:.0f},{y:.0f})',
                    (cx-30, cy-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,255), 2)
                cv2.circle(frame, (cx,cy), 8, (0,200,255), -1)
        self.pub_loads.publish(String(data=json.dumps(loads)))

        # ── Drop zones (ID 20 and 21) ─────────────────────────────
        drop_colors = {20: (0,255,255), 21: (255,0,255)}  # cyan / magenta
        drop_pubs   = {20: self.pub_drop1, 21: self.pub_drop2}
        drop_labels = {20: 'DROP-1', 21: 'DROP-2'}

        for i, mid in enumerate(ids_flat):
            if mid in DROPZONE_IDS:
                c    = corners[i][0]
                x, y = self._to_mm(np.mean(c[:,0]), np.mean(c[:,1]))
                drop_pubs[mid].publish(String(data=json.dumps(
                    {'x_mm': round(x,1), 'y_mm': round(y,1)})))

                cx, cy = int(np.mean(c[:,0])), int(np.mean(c[:,1]))
                color  = drop_colors[mid]

                # Draw 300x300mm square
                tl = mm_to_px(x - DROP_HALF_MM, y - DROP_HALF_MM)
                tr = mm_to_px(x + DROP_HALF_MM, y - DROP_HALF_MM)
                br = mm_to_px(x + DROP_HALF_MM, y + DROP_HALF_MM)
                bl = mm_to_px(x - DROP_HALF_MM, y + DROP_HALF_MM)
                pts = np.array([tl, tr, br, bl], dtype=np.int32)
                cv2.polylines(frame, [pts], isClosed=True,
                              color=color, thickness=2)
                cv2.circle(frame, (cx,cy), 6, color, -1)
                cv2.putText(frame,
                    f'{drop_labels[mid]} ({x:.0f},{y:.0f})',
                    (cx-30, cy-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ── HUD ───────────────────────────────────────────────────
        hud = f'Markers:{len(ids_flat)}  Bots:{len(bots)}  Loads:{len(loads)}'
        cv2.putText(frame, hud, (10, frame.shape[0]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self._show(frame)

    def _show(self, frame):
        if self.record and self.writer:
            self.writer.write(frame)
        cv2.imshow('MARS ArUco', frame)
        cv2.waitKey(1)

    def _to_mm(self, px, py):
        pt  = np.array([[[px, py]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self.H)
        return float(out[0][0][0]), float(out[0][0][1])

    def _yaw(self, c):
        dx = c[1][0] - c[0][0]
        dy = c[1][1] - c[0][1]
        return float(np.degrees(np.arctan2(-dy, dx)))

    def destroy_node(self):
        if self.record and self.writer:
            self.writer.release()
            self.get_logger().info('Recording saved.')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()