import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

angle = 0.71

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_finder') 

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.sub_fiction = self.create_subscription(Bool, '/mode', self.fiction_topic, 10)
        #self.pub_fiction = self.create_publisher(Bool, '/mode', 10)
        self.subscription = 0
        self.cnt_obstacles = 0


    def set_odom(self):
        self.subscription = self.create_subscription(
           Odometry,
           'odom',
           self.odom_callback,
           10
        )

        self.current_distance = 0.0  # current distance travelled
        self.current_angle = 0.0  # current angle turned

    def fiction_topic(self):
    # Move the robot
        self.move_robot(.52, angle=.58)

        self.subscription_laser = self.create_subscription(
           LaserScan, 
           '/scan', 
           self.lidar_callback, 
           10)

        while (self.cnt_obstacles <= 2):
            rclpy.spin_once(self)

        #self.move_robot(.4, angle=.4)
        vel = Twist()
        vel.angular.z = 1.0
        vel.linear.x = .2
        self.publisher_.publish(vel)
        time.sleep(2)
        vel.angular.z = .0
        vel.linear.x = .0
        self.publisher_.publish(vel)
        


    def odom_callback(self, msg):
        # Update current distance and angle
        self.current_distance = msg.pose.pose.position.x
        self.current_angle = msg.pose.pose.orientation.z

    def move_robot(self, distance, angle):
        self.set_odom()
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

        if self.cnt_obstacles == 3:
            return
        
        vel = Twist()
        global angle
        timer = 2
        forward_range =  np.array(msg.ranges)

        print(self.cnt_obstacles)

        last_10_elements = forward_range[-10:]
        first_10_elements = forward_range[:10] 

        forward_range = np.concatenate((last_10_elements, first_10_elements))
        if np.min(forward_range) > 0.2:
            vel.linear.x = 0.2
            self.publisher_.publish(vel)
        else:
            
            self.cnt_obstacles += 1

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
            twist_to_straight.angular.z = angle+.07
            self.publisher_.publish(twist_to_straight)
            time.sleep(timer)

            angle *= -1

            straight_movement.linear.x = 0.16
            self.publisher_.publish(straight_movement)
            time.sleep(timer)

def main(args=None):
    rclpy.init(args=args)

    circle_publisher = CirclePublisher()

    circle_publisher.fiction_topic()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()