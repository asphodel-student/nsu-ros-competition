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
        # self.control_subscribtion = self.create_subscription(ControlMsg, '/control_tl', self.shutdown_callback, 10)
        
        self.publisher = self.create_publisher(TrafficLight, '/traffic_light', 1)

        self._bridge = CvBridge()

    def detect_traffic_light_color_callback(self, image_msg):
        """
        Функция, проверяющая светофор на наличие разрешющего сигнала.
        Реализована самым простейшим образом.
        При обнаружении зеленого цвета, отправляет сообщение в главную ноду и уничтожается
        """
        frame = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")

        target_color = np.array([2, 110, 2])
        mask = np.all(frame == target_color, axis=-1)

        if np.any(mask == True):
            self.get_logger().info('Green light!!!!')
            self.is_green_light = True
            msg = TrafficLight()
            msg.is_green = True
            self.publisher.publish(msg)
            self.destroy_node()

    def shutdown_callback(self, msg):
        state = msg.mode

        if state == False:
            self.get_logger().info("Shutting down...")
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()

    node = TrafficLightDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

main()
