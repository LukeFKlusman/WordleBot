import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.create_timer(1/30.0, self.publish_frame)
        self.get_logger().info('Webcam publishing on /camera/camera/color/image_raw')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

rclpy.init()
node = WebcamNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
