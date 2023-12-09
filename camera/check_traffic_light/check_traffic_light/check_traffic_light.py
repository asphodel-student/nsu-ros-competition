import os
import cv2
import numpy as np
import copy
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from messages.msg import ControlMsg
from messages.msg import TrafficLight

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.subscribtion = self.create_subscription(Image, '/color/image', self.detect_traffic_light_color_callback, 10)
        self.control_subscribtion = self.create_subscription(ControlMsg, '/control_tl', self.shutdown_callback, 10)
        self.publisher = self.create_publisher(TrafficLight, '/traffic_light', 1)

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
                    msg = TrafficLight()
                    msg.is_green = True
                    self.publisher.publish(msg)


    def shutdown_callback(self, msg):
        state = msg.mode

        if state == False:
            self.node.get_logger().info("Shutting down...")
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()

    node = TrafficLightDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

main()
