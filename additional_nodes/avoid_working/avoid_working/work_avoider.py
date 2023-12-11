import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

angle = 0.65

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_finder')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.right = True


    def lidar_callback(self, msg):
        vel = Twist()
        global angle
        timer = 2
        forward_range =  np.array(msg.ranges[20:80])
        if np.min(forward_range) > 0.2:
            vel.linear.x = 0.2
            self.publisher_.publish(vel)
        else:
            stop_movement = Twist()
            self.publisher_.publish(stop_movement)

            straight_movement = Twist()
            straight_movement.linear.x = -0.165
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

            vel.angular.z = -angle
            self.publisher_.publish(vel)
            time.sleep(timer)

            straight_movement.linear.x = 0.2
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

            twist_to_straight = Twist() 
            twist_to_straight.angular.z = angle
            self.publisher_.publish(twist_to_straight)
            time.sleep(timer)

            angle *= -1

            straight_movement.linear.x = 0.2
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

def main(args=None):
    rclpy.init(args=args)

    circle_publisher = CirclePublisher()

    rclpy.spin(circle_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()