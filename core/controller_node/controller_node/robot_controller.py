import rclpy
from rclpy.node import Node

from messages.msg import TrafficSign
from messages.msg import TrafficLight
from messages.msg import ControlMsg

class Controller(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.traffic_sign_detector_subscription = self.create_subscription(TrafficSign, '/sign_topic', self.get_traffic_sign, 10)
        self.traffic_light_checker_subscription = self.create_subscription(TrafficLight, '/traffic_light', self.get_traffic_sign, 10)

        self.pid_control = self.create_publisher(ControlMsg, '/pid_control', 1)
        self.sign_detector_control = self.create_publisher(ControlMsg, '/sign_controller', 1)
        self.tl_detector_control = self.create_publisher(ControlMsg, '/tl_control', 1)

        self.is_allowed_to_move = False
        self.current_traffic_sign = -1


    def global_callback(self, Image):
        """
        
        """
        pass

    def get_traffic_sign(self, sign_msg):
        self.current_traffic_sign = sign_msg.trafic_sign

        print(self.current_traffic_sign)


    def get_traffic_light_color(self, msg):
        if msg.is_green == True:
            self.is_allowed_to_move = True

            # Turn off the light checker node
            self.shutdown_light_checker()

            # Turn off the pid
            self.turn_on_pid()

            # Turn off the sign detector
            self.turn_on_sign_detector()


    def shutdown_pid(self):
        """
        Позволяет отключить ноду автоматического управления роботом
        """
        msg = ControlMsg()
        msg.mode = False
        self.pid_control.publish(msg)

    def turn_on_pid(self):
        """
        Включает ноду автоматического управления роботом
        """
        msg = ControlMsg()
        msg.mode = True

        if self.is_allowed_to_move == True:
            self.pid_control.publish(msg)

    def shutdown_sign_detector(self):
        """
        Позволяет выключить ноду распознавания знаков
        """
        msg = ControlMsg()
        msg.mode = False
        self.sign_detector_control.publish(msg)

    def turn_on_sign_detector(self):
        """
        Включает ноду распознавания знаков
        """
        msg = ControlMsg()
        msg.mode = True
        self.sign_detector_control.publish(msg)

    def shutdown_light_checker(self):
        """
        Позволяет выключить ноду распознавания сигналов светофора
        """
        msg = ControlMsg()
        msg.mode = False
        self.tl_detector_control.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
