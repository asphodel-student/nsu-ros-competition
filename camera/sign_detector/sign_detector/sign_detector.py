import os
import cv2
import numpy as np
import copy
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from messages.msg import TrafficSign
from messages.msg import ControlMsg

Signs = Enum('Signs', ['CROSSWALK', 'PARKING', 'TUNNEL', 'CROSS', 'LEFT_TURN', 'RIGHT_TURN', 'WORK'])

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detecror')

        self._bridge = CvBridge()
        self._sift = cv2.SIFT_create()
        self._flann = cv2.BFMatcher()

        dir_path = '/data/'

        self._all_signs = sorted(os.listdir('robot_app/src/camera/files/'))

        print(self._all_signs)
        self._signs_images = [cv2.imread(dir_path + image, cv2.IMREAD_GRAYSCALE) for image in self._all_signs]

        self._kp = []
        self._des = []
        for image in self._signs_images:
            if image is None:
                continue
            kp, des = self._sift.detectAndCompute(image, None)
            self._kp.append(kp)
            self._des.append(des)

        self.sign = -1
        self.mode = True

        self.subscriber = self.create_subscription(Image, '/color/image', self.detect_traffic_sign_callback, 10)
        self.sign_publisher = self.create_publisher(TrafficSign, '/sign_topic', 1)

        # Control topic
        self.controller_subscriber = self.create_subscription(ControlMsg, '/sign_controller', self.check_mode_callback, 10)
        
    def check_mode_callback(self, msg):
        self.mode = msg.mode

    def detect_traffic_sign_callback(self, image_data):
        """
        
        """

        if self.mode == False:
            return
        
        MIN_MATCH_COUNT = 50

        image = self._bridge.imgmsg_to_cv2(image_data, 'mono8')
        kp1, des1 = self._sift.detectAndCompute(image, None)

        matches, confidence = [], []
        for current_des in self._des:
            matches.append(self._flann.knnMatch(des1, current_des, k=2))

        for i, match in enumerate(matches):
            match_points = []
            for m, n in match:
                if m.distance < 0.7 * n.distance:
                    match_points.append(m)

            if len(match_points) > MIN_MATCH_COUNT:
                confidence.append((i, len(match_points)))

        if len(confidence) != 0:
            index = max(confidence, key=lambda x: x[1])[0]
            old = copy.copy(self.sign)
            self.sign = self.set_current_sign(index)

            if old != self.sign:
                print(self.sign)
                msg = TrafficSign()
                msg.trafic_sign = index
                self.sign_publisher.publish(msg)
               
            
    def set_current_sign(self, index: int) -> int:
        _dict = { 0: Signs.CROSSWALK, 1: Signs.LEFT_TURN, 2: Signs.PARKING, 3: Signs.CROSS, 4: Signs.TUNNEL , 5: Signs.WORK}

        return _dict[index]
    

def main():
    rclpy.init()

    node = SignDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

main()

    

