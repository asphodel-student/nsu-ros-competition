import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

from messages.msg import ControlMsg

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_following')
        self.img_sub = self.create_subscription(Image, '/color/image', self.subs_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller_subscribtion = self.create_subscription(ControlMsg, '/pid_control', self.check_mode_callback, 10)
        self.update_timer = self.create_timer(0.01, self.update_callback)
        self.bridge = CvBridge()

        self.frame = None
        self.gray = None
        self.dst = None
        self.prevpt1 = np.array([150, 40])
        self.prevpt2 = np.array([560, 40])
        self.error = 0

        # This node works when this flag is True
        self.mode = False

    def subs_callback(self, msg):
        if self.mode == False:
            return
        
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        target_color = np.array([218, 218, 218])

        mask = np.all(self.frame == target_color, axis=-1)

        self.frame[mask] = [0, 0, 0]

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        _, self.gray = cv2.threshold(self.gray, 220, 255, cv2.THRESH_BINARY)
       
        height, width = self.gray.shape
        left_border = int(0.2 * width)
        right_border = int(0.8 * width)

        self.gray[:, :left_border] = 0  
        self.gray[:, right_border:] = 0 

        self.dst = self.gray[self.gray.shape[0] // 3 * 2:, :]

        if self.dst.dtype != np.uint8:
            self.dst = self.dst.astype(np.uint8)
        retval, labels, stats, centroids = cv2.connectedComponentsWithStats(self.dst)
      
        if retval > 1:
            mindistance1 = []
            mindistance2 = []

            for p in centroids:
                ptdistance1 = np.abs(p - self.prevpt1)
                ptdistance2 = np.abs(p - self.prevpt2)
                mindistance1.append(ptdistance1[0])
                mindistance2.append(ptdistance2[0])

            threshdistance1 = min(mindistance1)
            threshdistance2 = min(mindistance2)

            minlb1 = np.argmin(mindistance1)
            minlb2 = np.argmin(mindistance2)

            cpt1 = (centroids[minlb1, 0], centroids[minlb1, 1])
            cpt2 = (centroids[minlb2, 0], centroids[minlb2, 1])

            if threshdistance1 > 100:
                cpt1 = self.prevpt1
            if threshdistance2 > 100:
                cpt2 = self.prevpt2

        else:
            cpt1 = self.prevpt1
            cpt2 = self.prevpt2

        self.prevpt1 = np.array(cpt1)
        self.prevpt2 = np.array(cpt2)

        fpt = ((cpt1[0] + cpt2[0]) / 2, (cpt1[1] + cpt2[1]) / 2 + self.gray.shape[0] // 3 * 2)
        cv2.cvtColor(self.dst, cv2.COLOR_GRAY2BGR)

        # cv2.circle(self.frame, (int(fpt[0]), int(fpt[1])), 2, (0, 0, 255), 2)
        # cv2.circle(self.dst, (int(cpt1[0]), int(cpt1[1])), 2, (0, 0, 255), 2)
        # cv2.circle(self.dst, (int(cpt2[0]), int(cpt2[1])), 2, (255, 0, 0), 2)

        cv2.circle(self.dst, (int(self.prevpt1[0]), int(self.prevpt1[1])), 2, 122, 2)
        cv2.circle(self.dst, (int(self.prevpt2[0]), int(self.prevpt2[1])), 2, 122, 2)

        self.error = self.dst.shape[1] // 2 - fpt[0]

        # if self.error < 0:
        #     self.error = -self.error

        # self.get_logger().info(str(self.error))

        cv2.imshow("camera", self.gray)
        cv2.imshow("gray", self.dst)
        cv2.waitKey(1)

    def update_callback(self):
        cmd_vel = Twist()

        if self.mode:
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = (self.error * 90.0 / 400) / 16

            self.cmd_vel_pub.publish(cmd_vel)

    def check_mode_callback(self, msg):
        self.mode = msg.mode


def main(args=None):
    rclpy.init(args=args)

    node = LineFollower()

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
