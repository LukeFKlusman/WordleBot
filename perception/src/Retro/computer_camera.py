import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class WordlePerceptionNode(Node):
    def __init__(self):
        super().__init__('wordle_perception')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # 1. Convert ROS Image to OpenCV
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # 2. Convert to HSV for better color filtering
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        # 3. Define range for Wordle Green
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # 4. Create a mask and find contours
        mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500: # Filter small noise
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(current_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        cv2.imshow("Wordle Detection", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = WordlePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()