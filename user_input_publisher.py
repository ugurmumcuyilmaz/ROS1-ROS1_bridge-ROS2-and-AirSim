import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, 'user_input', 10)
        self.throttle = 0
        self.steering = 0
        self.gear = 1

    def get_user_input(self):
        msg = String()
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == 'w':
                    self.throttle += 0.1
                    msg.data = "FORWARD"
                elif key == 's':
                    self.throttle -= 0.1
                    msg.data = "BACKWARD"
                elif key == 'd':
                    self.steering = min(self.steering + 0.1, 1.0)  # rechts
                    msg.data = "RIGHT"
                elif key == 'a':
                    self.steering = max(self.steering - 0.1, -1.0)  # links
                    msg.data = "LEFT"
                elif key == ' ':  # stop
                    self.throttle = 0
                    msg.data = "STOP"
                elif key == 'q':  # q exit
                    msg.data = "QUIT"
                    self.publisher_.publish(msg)
                    return
                else:
                    self.throttle = max(0, self.throttle - 0.1)
                    msg.data = "STOP"

                self.throttle = min(1.0, self.throttle)

                # reverse
                if self.throttle < 0 and self.gear != -1:
                    self.gear = -1
                elif self.throttle >= 0 and self.gear == -1:
                    self.gear = 1

                car_msg = f"Throttle: {self.throttle}, Steering: {self.steering}, Gear: {self.gear}"
                msg.data = car_msg
                self.publisher_.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    user_input_publisher = UserInputPublisher()
    user_input_publisher.get_user_input()
    rclpy.spin(user_input_publisher)
    user_input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

