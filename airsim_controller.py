import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import airsim

class AirSimController(Node):
    def __init__(self):
        super().__init__('airsim_controller')
        self.subscription = self.create_subscription(
            String,
            'user_input',
            self.control_callback,
            10)
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.throttle = 0.0
        self.steering = 0.0
        self.gear = 1  # Am anfang gear=1

    def control_callback(self, msg):
        car_controls = airsim.CarControls()
        data = msg.data.split(", ")
        throttle = float(data[0].split(": ")[1])
        steering = float(data[1].split(": ")[1])
        gear = int(data[2].split(": ")[1])

        car_controls.throttle = throttle
        car_controls.steering = steering
        car_controls.is_manual_gear = True
        car_controls.manual_gear = gear

        self.client.setCarControls(car_controls)


def main(args=None):
    rclpy.init(args=args)
    airsim_controller = AirSimController()
    rclpy.spin(airsim_controller)
    airsim_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

