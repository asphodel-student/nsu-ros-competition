import os
import cv2
import numpy as np
import copy
from enum import Enum

import pkg_resources

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

from messages.msg import TrafficSign
from messages.msg import ControlMsg

from ament_index_python.packages import get_package_share_directory

import os

Signs = {0: 'CROSSWALK', 1: 'LEFT_TURN', 2: 'PARKING', 3: 'TUNNEL', 4: 'RIGHT_TURN', 5: 'WORK'}

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        self._bridge = CvBridge()
        self._sift = cv2.SIFT_create()

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        self._flann = cv2.FlannBasedMatcher(index_params, search_params)

        package_share_directory = get_package_share_directory('sign_detector')
        dir_path = os.path.join(package_share_directory, 'data/')

        self._all_signs = sorted(os.listdir(dir_path))

        self._signs_images = [cv2.imread(dir_path + image, cv2.IMREAD_GRAYSCALE) for image in self._all_signs]

        self._kp = []
        self._des = []
        for image in self._signs_images:
            if image is None:
                continue
            kp, des = self._sift.detectAndCompute(image, None)
            self._kp.append(copy.copy(kp))
            self._des.append(copy.copy(des))

        # image = cv2.imread(dir_path + 'turn_right.png', cv2.IMREAD_GRAYSCALE)
        # self._kp, self._des = self._sift.detectAndCompute(image, None)

        self.sign = -1
        self.mode = True

        self.subscriber = self.create_subscription(Image, '/color/image', self.detect_traffic_sign_callback, 10)
        self.pub = self.create_publisher(Image, '/signs_image', 1)
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
        
        MIN_MATCH_COUNT = 20

        image = self._bridge.imgmsg_to_cv2(image_data, 'mono8')
        kp1, des1 = self._sift.detectAndCompute(image, None)

        matches, confidence = [], []
        for current_des in self._des:
            matches.append(self._flann.knnMatch(des1, current_des, k=2))

        for i, match in enumerate(matches):
            match_points = []
            for m, n in match:
                if m.distance < 0.55 * n.distance:
                    match_points.append(m)

            if len(match_points) > MIN_MATCH_COUNT:
                confidence.append((i, len(match_points), copy.copy(match_points)))

        if len(confidence) != 0:
            index, length, data = max(confidence, key=lambda x: x[1])

            self.get_logger().info('POINTS: {}'.format(length))

            src_pts = np.float32([ kp1[m.queryIdx].pt for m in data ]).reshape(-1,1,2)
            dst_pts = np.float32([ self._kp[index][m.trainIdx].pt for m in data ]).reshape(-1,1,2)

            # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            # matchesMask_tunnel = mask.ravel().tolist()

            #mse = self.fnCalcMSE(src_pts, dst_pts)
            #self.get_logger().info('{}'.format(mse))
            #if mse < 100000:
            old = copy.copy(self.sign)
            self.sign = self.set_current_sign(index)

            if old != self.sign:
                self.get_logger().info(str(self.sign))

                msg = TrafficSign()
                msg.trafic_sign = self.sign
                self.sign_publisher.publish(msg)

            # draw_params_tunnel = dict(matchColor = (255,0,0), 
            #                 singlePointColor = None,
            #                 matchesMask = matchesMask_tunnel, 
            #                 flags = 2)

            # final_tunnel = cv2.drawMatches(image,kp1,self._signs_images[index], self._kp[index], data, None, **draw_params_tunnel)
            # self.pub.publish(self._bridge.cv2_to_imgmsg(final_tunnel, "bgr8"))

        # matches = self._flann.knnMatch(des1, self._des, k=2)

        # match_points = []
        # for m, n in matches:
        #     if m.distance < 0.7 * n.distance:
        #         match_points.append(m)

        # if len(match_points) > MIN_MATCH_COUNT:
        #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in match_points ]).reshape(-1,1,2)
        #     dst_pts = np.float32([ self._kp[m.trainIdx].pt for m in match_points ]).reshape(-1,1,2)

        #     # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        #     # matchesMask_tunnel = mask.ravel().tolist()

        #     mse = self.fnCalcMSE(src_pts, dst_pts)

        #     self.get_logger().info('POINTS: {}'.format(len(match_points)))
        #     self.get_logger().info('{}'.format(mse))

        #     if mse < 100000:
        #         old = copy.copy(self.sign)
        #         self.sign = self.set_current_sign(5)

        #         if old != self.sign:
        #             self.get_logger().info(str(self.sign))

        #             msg = TrafficSign()
        #             msg.trafic_sign = self.sign
        #             self.sign_publisher.publish(msg)

        # draw_params_tunnel = dict(matchColor = (255,0,0), 
        #                 singlePointColor = None,
        #                 matchesMask = matchesMask_tunnel, 
        #                 flags = 2)

        # final_tunnel = cv2.drawMatches(image,kp1,self._signs_images[5], self._kp[5], match_points, None, **draw_params_tunnel)
        # self.pub.publish(self._bridge.cv2_to_imgmsg(final_tunnel, "bgr8"))
            
            
    def set_current_sign(self, index: int) -> str:
        return Signs[index]
    
    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
        err = sum / num_all
        return err
    

def main():
    rclpy.init()

    node = SignDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

main()

    

