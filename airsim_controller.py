import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
import airsim

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')
        self.throttle_subscription = self.create_subscription(
            Float64,
            'throttle',
            self.throttle_callback,
            10)
        self.steering_subscription = self.create_subscription(
            Float64,
            'steering',
            self.steering_callback,
            10)
        self.brake_subscription = self.create_subscription(
            Float64,
            'brake',
            self.brake_callback,
            10)
        self.handbrake_subscription = self.create_subscription(
            Bool,
            'handbrake',
            self.handbrake_callback,
            10)
        self.gear_subscription = self.create_subscription(
            String,
            'gear',
            self.gear_callback,
            10)
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.handbrake = False
        self.gear = 'neutral'
    
    def throttle_callback(self, msg):
        self.throttle = msg.data
        self.send_control_command()
    
    def steering_callback(self, msg):
        self.steering = msg.data
        self.send_control_command()
    
    def brake_callback(self, msg):
        self.brake = msg.data
        self.send_control_command()

    def handbrake_callback(self, msg):
        self.handbrake = msg.data
        self.send_control_command()

    def gear_callback(self, msg):
        self.gear = msg.data
        self.send_control_command()
    
    def send_control_command(self):
        car_controls = airsim.CarControls()
        car_controls.throttle = self.throttle
        car_controls.steering = self.steering
        car_controls.brake = self.brake
        car_controls.handbrake = self.handbrake
        car_controls.is_manual_gear = True
        car_controls.manual_gear = 1 if self.gear == 'drive' else -1 if self.gear == 'reverse' else 0
        if self.brake > 0.0:
            car_controls.throttle = 0.0  # Ensure throttle is zero when braking
        self.client.setCarControls(car_controls)

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
