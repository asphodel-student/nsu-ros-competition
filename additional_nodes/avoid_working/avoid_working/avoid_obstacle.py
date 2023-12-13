import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

angle = 0.3

class Obstacle_Avoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider') 
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        self.subscription_laser = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10)
        self.subscription_odom= self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def lidar_callback(self, msg):
        view_range =  np.array(msg.ranges)

        right_view = (view_range[-40:])[-20:]
        left_view = (view_range[40:])[-20:]
        
        front_view = np.concatenate(view_range[20:], view_range[:-20])

        



def main(args=None):
    rclpy.init(args=args)

    node = Obstacle_Avoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()