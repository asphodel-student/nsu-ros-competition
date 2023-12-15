import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from messages.msg import ControlMsg

angle = 0.69

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_finder')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.control_subscriber = self.create_subscription(ControlMsg, '/obstacle_control', self.check_mode_callback, 10)

        self.mode = False
        self.obstacle_avoidence = 0

    def check_mode_callback(self, msg):
        self.mode = msg.mode

    def lidar_callback(self, msg):
        if not self.mode:
            return
        
        vel = Twist()

        if self.obstacle_avoidence > 3:

            vel.angular.z = 1.0
            vel.linear.x = .2
            self.publisher_.publish(vel)
            time.sleep(2)
            vel.angular.z = .0
            vel.linear.x = .0
            self.publisher_.publish(vel)
            self.destroy_node()
            rclpy.shutdown()

        if self.obstacle_avoidence == 0:

            vel.angular.z = 0.0
            vel.linear.x = 0.42
            self.publisher_.publish(vel)
            time.sleep(2)
            vel.angular.z = .87
            vel.linear.x = .0
            self.publisher_.publish(vel)
            time.sleep(2)
            vel.angular.z = .0
            vel.linear.x = .0
            self.publisher_.publish(vel)

            self.obstacle_avoidence += 1
        
        vel = Twist()
        global angle
        timer = 2
        forward_range =  np.array(msg.ranges)
        last_10_elements = forward_range[-10:]
        first_10_elements = forward_range[:10]
        
        forward_range = np.concatenate((first_10_elements, last_10_elements)) 

        if np.min(forward_range) > 0.2:
            vel.linear.x = 0.2
            self.publisher_.publish(vel)
        else:
            stop_movement = Twist()
            self.publisher_.publish(stop_movement)

            straight_movement = Twist()
            straight_movement.linear.x = -0.18
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

            vel.angular.z = - angle
            self.publisher_.publish(vel)
            time.sleep(timer)

            straight_movement.linear.x = 0.2
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

            twist_to_straight = Twist() 
            twist_to_straight.angular.z = angle + .09
            self.publisher_.publish(twist_to_straight)
            time.sleep(timer)

            angle *= -1

            straight_movement.linear.x = 0.16
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

            self.obstacle_avoidence += 1

def main(args=None):
    rclpy.init(args=args)

    circle_publisher = CirclePublisher()

    rclpy.spin(circle_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if __name__ == '__main__':
    main()