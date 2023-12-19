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
from messages.msg import ControlMsg

import rclpy
from nav_msgs.msg import Odometry
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Quaternion
from math import degrees
from sensor_msgs.msg import LaserScan

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class Publisher(Node):

    def __init__(self):
        super().__init__('tunnel')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pid_control = self.create_publisher(ControlMsg, 'pid_control', 1)
        self.turn_control = self.create_subscription(ControlMsg, '/turn_control', self.check_mode, 10)
        # self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Get odometry
        self.suka = self.create_subscription(Odometry, '/odom', self.get_odom, 10)

        # Turn our robot whet it is nessesary
        self.timer1 = self.create_timer(0.05, self.timer_callback)

        self.bridge = CvBridge()
        self.count = 0
        self.stage = 0
        self.wait = 0
        self.left = -1
        self.ask = 0
        self.left = 1
        self.check = 0
        msg = ControlMsg()
        msg.mode = False
        self.pid_control.publish(msg)

        self.start_angle = None
        self.start_x = None

        self.destination_angle = None
        self.is_need_to_turn = True

        self.direction = None
        self.stage = 'TURNING'

        self.second_turn = False

        self.pose = []
        self.orientation = []

        self.mode = False

    def check_mode(self, msg):
        self.mode = msg.mode

    def get_directiion(self):
        if self.direction == None:
            
            self.direction = 'RIGHT'

    def is_on_end_of_road(self):
        # self.get_logger().info('{}, {}'.format(self.pose.x, self.start_x))
        if abs(self.pose.x - self.start_x - 0.06) < 0.01:
            self.is_need_to_turn = True
        else:
            self.is_need_to_turn = False
    
    # def lidar_callback(self, msg):
    #     # if self.mode == False:
    #     #     return 
    #     # if self.check == 0:
    #     #     
    #     #     self.check = lidar_range[330]
    #     #     self.get_logger().info('{}'.format(self.check))
    #     lidar_range = np.array(msg.ranges) 

        # self.get_logger().info('{}, {}, {}, {}, {}, {}, {}, {},'.format(lidar_range[0], lidar_range[350], lidar_range[340], lidar_range[330], lidar_range[320], lidar_range[310], lidar_range[300], lidar_range[290]))



    def timer_callback(self):
        if self.mode == True:
            self.get_directiion()
            self.get_start_position()

            if self.stage == 'TURNING':
                if self.is_need_to_turn:
                    # self.get_logger().info('{}, {}'.format(self.destination_angle, self.orientation))
                    self.turn()
                else:
                    self.stage = 'DRIVING'
                    if self.second_turn == True:
                        self.mode = False

            # Turn on PID
            if self.stage == 'DRIVING':
                self.is_on_end_of_road()
                if self.is_need_to_turn == False:
                    msg = ControlMsg()
                    msg.mode = True
                    self.pid_control.publish(msg)
                else:
                    self.stage = 'TURNING'
                    self.start_angle = None
                    self.second_turn = True
                    msg = ControlMsg()
                    msg.mode = False
                    self.pid_control.publish(msg)

   
    def get_odom(self, msg):
        # self.get_logger().info('x: {}, w: {}'.format(msg.pose.pose.position.x, msg.pose.pose.orientation.w))
        self.pose = msg.pose.pose.position
        _, _, self.orientation = euler_from_quaternion(msg.pose.pose.orientation)

    def get_start_position(self):
        

        if self.start_angle == None:
            self.start_angle = self.orientation
            self.start_x = -0.09

            if self.direction == 'LEFT': 
                theta = 1.22
            elif self.direction == "RIGHT":
                theta = 0.4

            if self.second_turn == True and self.direction == "LEFT":
                theta = .9
            elif self.second_turn == True and self.direction == "RIGHT":
                theta = -0.8
            if self.direction == 'LEFT':
                self.destination_angle = self.start_angle + theta
            elif self.direction == 'RIGHT':
                 self.destination_angle = self.start_angle - theta
            if self.destination_angle > np.pi:
                self.destination_angle -= np.pi * 2

    def turn(self):
        if self.is_need_to_turn:
            if self.second_turn == True:
                self.destination_angle = 3.0

            if abs(self.destination_angle - self.orientation) > 0.01:
                twist = Twist()
                twist.linear.x = 0.08
                # twist.linear.x = 0.0
                # twist.angular.z = -0.32 if self.direction == 'RIGHT' else 0.32
                twist.angular.z = -0.3 if self.direction == 'RIGHT' else -0.3
                self.publisher.publish(twist)
            else:   
                self.is_need_to_turn = False
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)

                return



def main():
    rclpy.init()
    circling = Publisher()
    rclpy.spin(circling)
    rclpy.shutdown()

if __name__ == 'main':
    main()