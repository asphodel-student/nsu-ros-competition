import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
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
        twist.linear.x = 0.1  # linear velocity in x direction
        twist.angular.z = 0.0  # angular velocity around z axis
        # Drive the robot until the desired distance is achieved
        while self.current_distance < distance:
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self)  # allow callback to update current distance
        # Stop the robot after reaching the desired distance
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        # Rotate the robot until the desired angle is achieved
        twist.angular.z = 0.1  # angular velocity around z axis
        final_angle = self.current_angle + angle
        while self.current_angle < final_angle:
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self)  # allow callback to update current angle
        # Stop the robot after reaching the desired angle
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    # Specify desired distance and angle
    distance = 1.0
    angle = 1.57  # 90 degrees in radians
    # Move the robot
    robot_control.move_robot(distance, angle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()