import time
import sys
import math as m
from typing import Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Publisher(Node):

    def __init__(self):
        super().__init__('tunnel')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, "/depth/image", self.listener_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()
        self.is_obstacle = 0  
        self.speed = 0
        self.stage = 1
        self.angle = 250
        # self.angle = 250
        self.rotate = 0.5
        self.turn  = 0
        self.count = 0
        self.area  = 75
        
    def timer_callback(self):
        twist = Twist()
        if self.stage == 0:
            if self.count < 8:
                twist.linear.x = 0.2
                twist.angular.z = 0.1
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.stage = 1
        if self.stage == 1:
            if self.turn == 2:
                twist.linear.x = 0.08
                twist.angular.z = self.rotate
            elif self.turn == 1:
                twist.linear.x = 0.08
                twist.angular.z = - self.rotate
            else:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
        # twist.linear.x = 1.0 if not self.is_obstacle else 0.0
        self.publisher.publish(twist)
        self.count += 1

    def listener_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(f"CvBridgeError: {e}")
            return

        depth_array = np.array(depth_image, dtype=np.float32)
        h, w = depth_array.shape
        # print(depth_array[120])
        left, right = w // 2 - self.area - 1, w // 2 + self.area
        for i in range(w // 2 - self.area, w // 2 + self.area):
            if depth_array[self.angle][i] < 0.45:
                if left == i - 1:
                    left = i
                else:
                    right = i
                    break
        print(depth_array[self.angle][w // 2 - self.area - 1 : w // 2 + self.area])
        print(left, right)
        if left ==  w // 2 - self.area - 1 and right == w // 2 + self.area:
            self.turn = 0
        else:
            rotate_left = w // 2 - left
            rotate_right = right - w // 2
            if left == right - 1:
                if depth_array[self.angle][w // 2 - self.area - 1] < depth_array[self.angle][w // 2 + self.area]:
                    self.turn = 1
                else:
                    self.turn = 2
            else:
                if rotate_left > rotate_right:
                    self.turn = 2
                else:
                    self.turn = 1

def main():
    rclpy.init()
    circling = Publisher()
    rclpy.spin(circling)
    rclpy.shutdown()

if __name__ == 'main':
    main()