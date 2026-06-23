import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CAMERA_INDEX = 2

   # run: v4l2-ctl --list-devices  to find DroidCam
PUBLISH_HZ   = 30

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.pub    = self.create_publisher(Image, '/image_raw', 10)
        self.cap    = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {CAMERA_INDEX}')
            self.get_logger().error('Run: v4l2-ctl --list-devices')
            raise RuntimeError('Camera not found')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.get_logger().info('Camera ready → /image_raw')
        self.timer = self.create_timer(1.0/PUBLISH_HZ, self.tick)

    def tick(self):
        ret, frame = self.cap.read()
        if not ret: return
        # frame = cv2.flip(frame, 1)
        cv2.imshow('MARS Camera', frame)   # add this
        cv2.waitKey(1)  
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()