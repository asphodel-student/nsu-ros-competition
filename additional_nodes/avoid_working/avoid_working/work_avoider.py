import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


angle = 0.65

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_finder') 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_laser = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10)
        
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.current_distance = 0.0  # current distance travelled
        self.current_angle = 0.0  # current angle turned


    def odom_callback(self, msg):
        # Update current distance and angle
        self.current_distance = msg.pose.pose.position.x
        self.current_angle = msg.pose.pose.orientation.z

    def move_robot(self, distance, angle):
        # Create the twist message
        twist = Twist()
        twist.linear.x = 0.2  # linear velocity in x direction
        twist.angular.z = 0.0  # angular velocity around z axis
        # Drive the robot until the desired distance is achieved
        while self.current_distance < distance:
            self.publisher_.publish(twist)
            rclpy.spin_once(self)  # allow callback to update current distance
        # Stop the robot after reaching the desired distance
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        # Rotate the robot until the desired angle is achieved
        twist.angular.z = 0.5  # angular velocity around z axis
        final_angle = self.current_angle + angle

        while np.abs(self.current_angle) < np.abs(final_angle):
            self.publisher_.publish(twist)
            rclpy.spin_once(self)  # allow callback to update current angle
        # Stop the robot after reaching the desired angle
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    
    def lidar_callback(self, msg):
        vel = Twist()
        global angle
        timer = 2
        forward_range =  np.array(msg.ranges)

        last_10_elements = forward_range[-30:]
        first_10_elements = forward_range[:30] 

        forward_range = np.concatenate((last_10_elements, first_10_elements))
        if np.min(forward_range) > 0.2:
            vel.linear.x = 0.2
            self.publisher_.publish(vel)
        else:
            stop_movement = Twist()
            self.publisher_.publish(stop_movement)

            straight_movement = Twist()
            straight_movement.linear.x = -0.168
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

    distance = 0.5
    alpha = 0.5  # 90 degrees in radians
    # Move the robot
    circle_publisher.move_robot(distance, angle=alpha)

    rclpy.spin(circle_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()