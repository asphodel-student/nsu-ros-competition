import rclpy
from rclpy.node import Node

from messages.msg import TrafficSign
from messages.msg import TrafficLight
from messages.msg import CrosswalkCheck
from messages.msg import ControlMsg
from std_msgs.msg import Bool

class Controller(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.traffic_sign_detector_subscription = self.create_subscription(TrafficSign, '/sign_topic', self.get_traffic_sign, 10)
        self.traffic_light_checker_subscription = self.create_subscription(TrafficLight, '/traffic_light', self.get_traffic_light_color, 10)
        self.crosswalk_checker_subscription = self.create_subscription(CrosswalkCheck, '/crosswalk', self.get_crosswalk_status, 10)

        self.pid_control = self.create_publisher(ControlMsg, '/pid_control', 1)
        self.sign_detector_control = self.create_publisher(ControlMsg, '/sign_controller', 1)
        self.tl_detector_control = self.create_publisher(ControlMsg, '/tl_control', 1)
        self.crosswalk_control = self.create_publisher(ControlMsg, '/crosswalk_control', 1)
        self.obstacle_control = self.create_publisher(ControlMsg, '/obstacle_control', 1)

        self.update_timer = self.create_timer(0.01, self.global_callback)

        self.is_allowed_to_move = False
        self.current_traffic_sign = -1


    def global_callback(self):
        """
        
        """

        if self.current_traffic_sign == 'CROSSWALK':
            self.turn_on_node(self.crosswalk_control)

        if self.current_traffic_sign == 'WORK':
            self.shutdown_node(self.pid_control)
            self.turn_on_node(self.obstacle_control)
        

    def get_traffic_sign(self, sign_msg):
        self.current_traffic_sign = sign_msg.trafic_sign
        self.get_logger().info('Detect new traffic sign: {}'.format(self.current_traffic_sign))

    def get_crosswalk_status(self, msg):
        self.is_allowed_to_move = -msg.is_allowed_to_move_forward
        if msg.is_allowed_to_move_forward == True:
            self.shutdown_node(self.pid_control)
        else:
            self.turn_on_node(self.pid_control)

    def get_traffic_light_color(self, msg):
        """
        Проверка сигнала светофора
        """
        if msg.is_green == True:
            self.get_logger().info('Green light!')

            # Turn off the light checker node
            self.shutdown_node(self.tl_detector_control)

            # Turn off the pid
            self.turn_on_node(self.pid_control)
    
            # Turn on the sign detector
            self.turn_on_node(self.sign_detector_control)

            self.is_allowed_to_move = True


    def shutdown_node(self, control_topic):
        """
        Позволяет отключить ноду 
        """
        msg = ControlMsg()
        msg.mode = False
        control_topic.publish(msg)

    def turn_on_node(self, control_topic):
        """
        Позволяет включить ноду
        """
        msg = ControlMsg()
        msg.mode = True
        control_topic.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
