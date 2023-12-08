import os
import cv2
import numpy as np
import copy
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.subscribtion = self.create_subscription(Image, '/color/image', self.detect_traffic_light_color_callback, 10)
        # self.publisher = self.create_publisher(Image, '/color/image', self.detect_traffic_light_color_callback, 10)

        self.is_green_light = False
        self.future = None

    def detect_traffic_light_color_callback(self, image_msg):

        frame = self.bridge.imgmsg_to_cv2(image_msg, "mono8")
        blurred = cv2.GaussianBlur(frame, (21, 21), 0)
        edges = cv2.Canny(blurred, 25, 30)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) > 6:
                ellipse = cv2.fitEllipse(contour)
                major_axis, minor_axis = max(ellipse[1]), min(ellipse[1])
                if minor_axis == 0:
                    continue
                
                center_pixel = tuple(map(int, ellipse[0]))
                color = frame[center_pixel[1], center_pixel[0]]

                is_green = all(abs(a - b) < 5 for a, b in zip(color, [0, 117, 0]))
                if is_green:
                    self.is_green_light = True
                    self.future.set_result(None)

    def shutdown_callback(self, msg):
        state = msg.data

        if state == 'shutdown':
            self.node.get_logger().info("Shutting down...")
            self.node.destroy_node()
            rclpy.shutdown()
