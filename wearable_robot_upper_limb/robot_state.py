#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wearable_robot_upper_limb_msgs.msg import UpperLimbState
import serial

OUTPUT_SERIALPORT                  = '/dev/ttyAMA0'

class UpperLimbStateOutNode(Node):
    def __init__(self):
        super().__init__('wearable_robot_upper_limb_state_out')
        
        # Subscribers
        self.upper_limb_state_sub = self.create_subscription(UpperLimbState, 'upper_limb_status', self.upper_limb_state_callback, 10)
        self.serial = serial.Serial(OUTPUT_SERIALPORT, 115200)
        
    def stop(self):
        self.get_logger().info('Stopping...')
 
    def upper_limb_state_callback(self, state_msg):
        state = state_msg.state
        self.r = state_msg.repeat
        loadcell_value = state_msg.weight
        position = state_msg.angle
        current = state_msg.current

        # State : Flexion , Repeat : 6 , Weight : 475.40g , Elbow Angle : 125.22° , Current : -304.0000mA
        # State : Extension , Repeat : 5 , Weight : -92.22g , Elbow Angle : 151.98° , Current : 46.0000mA
        output = f'State : {state}, Repeat : {self.r}, Weight : {loadcell_value:.2f}g, Elbow Angle : {position:.2f}°, Current : {current:.4f}mA\r\n'
        self.get_logger().info(output)
        self.serial.write(output.encode())

        
def main(args=None):
    rclpy.init(args=args)
    upper_limb_state_out_node = UpperLimbStateOutNode()

    try:
        rclpy.spin(upper_limb_state_out_node)
    except KeyboardInterrupt:
        upper_limb_state_out_node.get_logger().error('Keyboard Interrupt received')

    upper_limb_state_out_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
