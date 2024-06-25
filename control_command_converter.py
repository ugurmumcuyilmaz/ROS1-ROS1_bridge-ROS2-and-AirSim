import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool

class CarControlNode(Node):
    def __init__(self):
        super().__init__('car_control_node')
        self.subscription = self.create_subscription(
            String,
            'driver_input',
            self.listener_callback,
            10)
        self.throttle_publisher = self.create_publisher(Float64, 'throttle', 10)
        self.steering_publisher = self.create_publisher(Float64, 'steering', 10)
        self.brake_publisher = self.create_publisher(Float64, 'brake', 10)
        self.handbrake_publisher = self.create_publisher(Bool, 'handbrake', 10)
        self.gear_publisher = self.create_publisher(String, 'gear', 10)
        self.current_gear = 'neutral'
        self.throttle_value = 0.0
        self.steering_value = 0.0
        self.brake_value = 0.0
        self.handbrake_value = False
        self.is_accelerating = False
        self.is_reversing = False
        self.is_turning_left = False
        self.is_turning_right = False

        self.timer = self.create_timer(0.1, self.publish_control_commands)
        self.acceleration_rate = 0.1  # Rate at which the speed increases
        self.deceleration_rate = 0.05  # Rate at which the speed decreases
        self.steering_rate = 0.1  # Rate at which the steering angle increases

    def listener_callback(self, msg):
        if msg.data == 'up':
            self.is_accelerating = True
            self.is_reversing = False
            self.current_gear = 'drive'
        elif msg.data == 'down':
            self.is_accelerating = False
            self.is_reversing = True
            self.current_gear = 'reverse'
        elif msg.data == 'left':
            self.is_turning_left = True
            self.is_turning_right = False
        elif msg.data == 'right':
            self.is_turning_right = True
            self.is_turning_left = False
        elif msg.data == 'release_up':
            self.is_accelerating = False
        elif msg.data == 'release_down':
            self.is_reversing = False
        elif msg.data == 'release_left':
            self.is_turning_left = False
        elif msg.data == 'release_right':
            self.is_turning_right = False
        elif msg.data == 'space':
            self.handbrake_value = not self.handbrake_value  # Toggle handbrake
            if not self.handbrake_value:
                self.brake_value = 0.0  # Reset brake value when handbrake is released
        else:
            self.is_accelerating = False
            self.is_reversing = False
            self.is_turning_left = False
            self.is_turning_right = False
    
    def update_speed_and_steering(self):
        if not self.handbrake_value:
            if self.is_accelerating:
                self.throttle_value = min(self.throttle_value + self.acceleration_rate, 1.0)
            elif self.is_reversing:
                self.throttle_value = min(self.throttle_value + self.acceleration_rate, 1.0)
            else:
                self.throttle_value = max(self.throttle_value - self.deceleration_rate, 0.0)
        else:
            self.throttle_value = max(self.throttle_value - self.deceleration_rate, 0.0)
            self.brake_value = min(self.brake_value + self.acceleration_rate, 1.0)

        if self.is_turning_left:
            self.steering_value = max(self.steering_value - self.steering_rate, -0.5)
        elif self.is_turning_right:
            self.steering_value = min(self.steering_value + self.steering_rate, 0.5)
        else:
            if self.steering_value > 0:
                self.steering_value = max(self.steering_value - self.steering_rate, 0.0)
            elif self.steering_value < 0:
                self.steering_value = min(self.steering_value + self.steering_rate, 0.0)

    def publish_control_commands(self):
        self.update_speed_and_steering()
        throttle = Float64()
        steering = Float64()
        brake = Float64()
        handbrake = Bool()
        gear = String()
        
        throttle.data = self.throttle_value
        steering.data = self.steering_value
        brake.data = self.brake_value
        handbrake.data = self.handbrake_value
        gear.data = self.current_gear
        
        self.throttle_publisher.publish(throttle)
        self.steering_publisher.publish(steering)
        self.brake_publisher.publish(brake)
        self.handbrake_publisher.publish(handbrake)
        self.gear_publisher.publish(gear)

def main(args=None):
    rclpy.init(args=args)
    node = CarControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
