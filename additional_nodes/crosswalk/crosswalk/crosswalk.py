import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

from messages.msg import CrosswalkCheck
from messages.msg import ControlMsg

class CrosswalkChecker(Node):
    def __init__(self):
        super().__init__('crosswalk')
        # self.get_logger().info('Crosswalk-checker start to work')

        self.control_subscriber = self.create_subscription(ControlMsg, '/crosswalk_control', self.check_mode_callback, 10)
        self.depth_camera_subscription = self.create_subscription(Image, '/depth/image', self.check_crosswalk_callback, 10)
        self.crosswalk_publisher = self.create_publisher(CrosswalkCheck, '/crosswalk', 1)

        self.bridge = CvBridge()
        self.mode = False

    def check_crosswalk_callback(self, msg):
        if self.mode == False:
            return
  
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        depth_threshold = 0.3  
        depth_image[depth_image > depth_threshold] = 0

        offset_in_px = 80
        a, b = msg.width // 2 - offset_in_px, msg.width // 2 + offset_in_px

        message = CrosswalkCheck()
        message.is_allowed_to_move_forward = bool(np.any(depth_image[msg.height // 2 - 50, a:b] > 0))
        
        self.get_logger().info('Crosswalk-checker status: {}'.format(message.is_allowed_to_move_forward))
        self.crosswalk_publisher.publish(message)

    def check_mode_callback(self, msg):
        self.mode = msg.mode


def main(args=None):
    rclpy.init(args=args)

    node = CrosswalkChecker()

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()


