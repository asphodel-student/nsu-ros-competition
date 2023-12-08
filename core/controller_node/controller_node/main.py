import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

from camera import TrafficLightDetector
from robot_controller import Controller


def main():
    rclpy.init()
    node = TrafficLightDetector()

    node.future = rclpy.create_future()
    rclpy.spin_until_future_complete(node, node.future)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



