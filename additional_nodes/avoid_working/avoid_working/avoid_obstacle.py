"""import rclpy
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

        self.current_distance = 0.0  # current distance travelled
        self.current_angle = 0.0  # current angle turned

    def lidar_callback(self, msg):
        view_range =  np.array(msg.ranges)

        right_view = (view_range[-80:])[-40:]
        left_view = (view_range[80:])[-40:]
        
        front_view = np.concatenate((view_range[:40], view_range[-40:]))

        views = [left_view, front_view, right_view]

        mean_views = [np.mean(view) for view in views]

        free_view = np.argmax(mean_views)
        print(mean_views)

        obstacle_view = np.argmin(mean_views)

        vel = Twist()

        if free_view == 2:
            vel.angular.z = angle
        elif free_view == 0:
            vel.angular.z = -angle
        else:
            vel.angular.z = 0.0
        
        self.publisher_.publish(vel)

    def odom_callback(self, msg):
        self.current_distance = msg.pose.pose.position.x
        self.current_angle = msg.pose.pose.orientation.z
        


def main(args=None):
    rclpy.init(args=args)

    node = Obstacle_Avoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        self.subscription = self.create_subscription(Image, '/color/image', self.callback_camera, 10)
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
    def callback_camera(self, image_msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        
        # Применить фильтры изображения для выделения белых и желтых объектов
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Определить диапазоны цветов для белых и желтых объектов
        white_lower = (0, 0, 200)
        white_upper = (255, 30, 255)
        yellow_lower = (15, 100, 100)
        yellow_upper = (35, 255, 255)
        
        # Создать маски для белых и желтых объектов
        white_mask = cv2.inRange(hsv_image, white_lower, white_upper)
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        
        # Применить маски к исходному изображению
        white_result = cv2.bitwise_and(cv_image, cv_image, mask=white_mask)
        yellow_result = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)
        
        # Найти контуры белых и желтых объектов
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Проверить, есть ли контуры белых или желтых объектов
        is_white_detected = len(white_contours) > 0
        is_yellow_detected = len(yellow_contours) > 0
        
        # Проверить, нужно ли остановить робота
        stop_robot_msg = is_white_detected or is_yellow_detected
        print(stop_robot_msg)
        vel = Twist()
        if stop_robot_msg:
            vel.linear.x = 0.0
        else:
            vel.linear.x = 1.0
        self.publisher.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()


