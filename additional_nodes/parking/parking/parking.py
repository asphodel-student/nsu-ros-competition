import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from messages.msg import ControlMsg
from nav_msgs.msg import Odometry
import threading

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

class Parking(Node):

    def __init__(self):
        super().__init__('parking')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.subscription = self.create_subscription(Image, "/depth/image", self.listener_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.control_topic = self.create_subscription(ControlMsg, '/parking_control', self.check_mode, 10)
        self.test = self.create_publisher(ControlMsg, '/pid_control', 1)

        self.timer = self.create_timer(0.5, self.timer_callback)

        # self.bridge = CvBridge()
        self.is_obstacle = 0  
        self.speed = 0
        self.stage = 0
        self.angle = 250

        self.rotate = 0.5
        
        self.count = 0
        self.area  = 75
        self.wait = 0

        self.x = 0.0
        self.y = 0.0
        self.orientation = None

        self.mode = False

        self.is_left_obstacle = False
        self.is_right_obstacle = False

        self.left_distanse = 0.0
        self.right_distanse = 0.0

        self.angles = [-np.pi / 2, 0, np.pi, np.pi / 2, np.pi]
        self.directrion = [1, 1, 1, 1]
        self.current_angle = 0

        self.is_start = False

    def check_mode(self, msg):
        self.mode = msg.mode

    def odom_callback(self, msg):
        if self.mode == False:
            return

        self.x = msg.pose.pose.position.x
        _, _, self.orientation = euler_from_quaternion(msg.pose.pose.orientation)


    def turn(self, destination_angle):
        # self.get_logger().info('{}, {}'.format(destination_angle, self.orientation))
        if self.orientation == None:
            return

        if abs(destination_angle - self.orientation) > 0.1:
            if self.current_angle != 0:
                # self.get_logger().info('CHANGED')
                if self.is_left_obstacle:
                    self.directrion[self.current_angle] *= -1

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.4 * self.directrion[self.current_angle]          

            self.publisher.publish(twist)
        else:   
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.is_need_to_move_forward = True
            self.is_need_to_turn = False
            self.current_angle += 1

    def move_forward(self):
        condition = False

        # self.get_logger().info('GAGAGAG: {}'.format(self.current_angle))

        # Доезжаем до парковочных мест   
        if self.current_angle == 1:
            condition = not(self.is_left_obstacle or self.is_right_obstacle)
            # self.get_logger().info('CCCCCCCCC {}'.format(condition))

        # Заезд
        elif self.current_angle == 2:
            condition = self.is_back_obstacle

        #

        #

        #

        twist = Twist()
        if condition:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.is_need_to_turn = True
            self.is_need_to_move_forward = False

        self.publisher.publish(twist)

    def lidar_callback(self, msg):
        if self.mode == False:
            return 

        lidar_range = np.array(msg.ranges) 

        back_size = lidar_range[176:186]
        left_side = lidar_range[80:100]
        right_side = lidar_range[260:2800]
        
        self.is_back_obstacle = np.any(back_size < 0.4)
        self.is_left_obstacle = np.all(left_side < 1.5)
        self.is_right_obstacle = np.all(right_side < 1.5)

        self.back_distanse = np.min(back_size)
        self.left_distanse = np.min(left_side)
        self.right_distanse = np.min(right_side)



    def timer_callback(self):
        if self.mode == False:
            return 





        if self.current_angle == len(self.angles):
            self.mode = False
            msg = ControlMsg()
            msg.mode = True
            self.test.publish(msg)
            # self.get_logger().info('PID WAS ENABLED')
            return

        if self.is_start == False:
            if abs(self.x + 0.32) < 0.05:
                # self.get_logger().info('BBBBBBBBB')
                self.is_start = True
                self.is_need_to_turn = True
                msg = ControlMsg()
                msg.mode = False
                self.test.publish(msg)
            
        if self.is_start:
            if (self.current_angle <= 1):
                if self.is_need_to_turn:
                    if self.current_angle < 1:
                        self.turn(self.angles[self.current_angle])
                else:
                    self.move_forward()




def main():
    rclpy.init()
    circling = Parking()
    rclpy.spin(circling)
    rclpy.shutdown()

if __name__ == 'main':
    main()