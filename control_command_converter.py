import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControlCommandConverter(Node):
    def __init__(self):
        super().__init__('control_command_converter')
        self.publisher_ = self.create_publisher(String, 'control_command', 10)
        self.subscription = self.create_subscription(
            String,
            'user_input',
            self.listener_callback,
            10)

    from std_msgs.msg import String

    def listener_callback(self, msg):
    	data = msg.data.split(", ")
    	throttle = float(data[0].split(": ")[1])
    	steering = float(data[1].split(": ")[1])
    	gear = int(data[2].split(": ")[1])

    	car_msg = f"Throttle: {throttle}, Steering: {steering}, Gear: {gear}"
    	
    	msg_to_publish = String()
    	msg_to_publish.data = car_msg
    	self.publisher_.publish(msg_to_publish)


def main(args=None):
    rclpy.init(args=args)
    control_command_converter = ControlCommandConverter()
    rclpy.spin(control_command_converter)
    control_command_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

