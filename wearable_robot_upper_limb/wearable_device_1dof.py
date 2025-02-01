import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition

class DynamixelPositionPublisher(Node):

    def __init__(self):
        super().__init__('wearable_device_upper_limb_controller')
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10)
        timer_period = 0.1  # seconds (10Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = 0
        self.direction = 1  # 1 for increasing, -1 for decreasing
        self.get_logger().info('Dynamixel Position Publisher Node has been started')

    def timer_callback(self):
        msg = SetPosition()
        msg.id = 1
        msg.position = self.position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID: {msg.id}, Position: {msg.position}')

        # Update position for next publish
        self.position += 10 * self.direction
        if self.position >= 1000:
            self.position = 1000
            self.direction = -1
        elif self.position <= 0:
            self.position = 0
            self.direction = 1

def main(args=None):
    rclpy.init(args=args)

    dynamixel_position_publisher = DynamixelPositionPublisher()

    rclpy.spin(dynamixel_position_publisher)

    dynamixel_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
