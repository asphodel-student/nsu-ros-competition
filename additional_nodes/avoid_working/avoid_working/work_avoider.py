import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
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

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_finder')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.control_subscriber = self.create_subscription(ControlMsg, '/obstacle_control', self.check_mode_callback, 10)
        self.control_publisher = self.create_publisher(ControlMsg, '/obstacle_control', 10)
        self.timer = self.create_timer(0.08, self.timer_callback)
        self.test = self.create_publisher(ControlMsg, '/pid_control', 1)
    
        self.mode = False
        self.range = 0
        self.pid_disabled = False

        self.is_forward_obstacle = False
        self.is_left_obstacle = False
        self.is_right_obstacle = False

        self.is_need_to_turn = True
        self.is_need_to_move_forward = False

        self.start = False

        self.current_angle = 0
        self.angles = [np.pi - 0.01, np.pi / 2, 0, np.pi / 2]
        self.directions = [1, -1, -1, 1]

        self.orientation = None
        self.x = 0.0

    def odom_callback(self, msg):
        if self.mode == False:
            return

        self.x = msg.pose.pose.position.x
        _, _, self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
        # self.get_logger().info('{}'.format(self.orientation))
        
    def check_mode_callback(self, msg):
        self.mode = msg.mode
        # self.get_logger().info('CHANGE MODE: {}'.format(self.mode))

    def turn(self, destination_angle):
        # self.get_logger().info('{}, {}'.format(destination_angle, self.orientation))
        if self.orientation == None:
            return
        if abs(destination_angle - self.orientation) > 0.1:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.4 * self.directions[self.current_angle]
            self.publisher.publish(twist)
        else:   
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.is_need_to_move_forward = True
            self.is_need_to_turn = False
            self.current_angle += 1

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
            
        if self.is_need_to_turn:
            self.turn(self.angles[self.current_angle])
        else:
            self.move_forward()

    def lidar_callback(self, msg):
        if self.mode == False:
            return 

        lidar_range = np.array(msg.ranges)

        last_10_elements = lidar_range[-10:]
        first_10_elements = lidar_range[:10]

        forward_range = np.concatenate((first_10_elements, last_10_elements))     

        left_side = lidar_range[82:96]
        right_side = lidar_range[268:276]

        self.is_forward_obstacle = np.any(forward_range < 0.22)
        self.is_left_obstacle = np.any(left_side < 0.22)
        self.is_right_obstacle = np.any(right_side < 0.22)

        self.forward_distanse = np.min(forward_range)
        self.left_distanse = np.min(left_side)
        self.right_distanse = np.min(right_side)

        if self.start is False and self.is_forward_obstacle:
            # self.get_logger().info('PID was shutdowned')
            msg = ControlMsg()
            msg.mode = False
            self.pid_dibasled = True
            self.control_publisher.publish(msg)
            self.start = True
        
    def move_forward(self):
        condition = False
        if self.current_angle == 1:
            condition = self.is_right_obstacle
        elif self.current_angle == 2:
            condition = not self.is_forward_obstacle
        elif self.current_angle == 3:
            # self.get_logger().info('IM HERE!')
            condition = self.is_left_obstacle

        twist = Twist()
        if condition:
            twist.linear.x = 0.18
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.is_need_to_turn = True
            self.is_need_to_move_forward = False

        self.publisher.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    circle_publisher = CirclePublisher()

    rclpy.spin(circle_publisher)

    circle_publisher.destroy_node()
    rclpy.shutdown()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if __name__ == 'main':
    main()