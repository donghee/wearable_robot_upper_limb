import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
from hx711 import HX711
import time

class LoadCellNode(Node):
    def __init__(self):
        super().__init__('load_cell_node')
        self.publisher_ = self.create_publisher(Float32, 'load_cell_weight', 10)
        self.timer = self.create_timer(1.0, self.publish_weight)  # Publish every 1 second
        
        self.get_logger().info("Initializing load cell...")
        self.init_load_cell()
        self.get_logger().info("Load cell initialized.")

    def init_load_cell(self):
        # Calibration
        measures = []
        try:
            hx711 = HX711(
                dout_pin=5,
                pd_sck_pin=6,
                channel='A',
                gain=64
            )
            hx711.reset()
            measures = hx711.get_raw_data(times=10)
        finally:
            GPIO.cleanup()

        self.OFFSET = sum(measures) / len(measures)
        self.SCALE_FACTOR = 100  # DONGHEE

    def read_weight(self):
        measures = []
        try:
            hx711 = HX711(
                dout_pin=5,
                pd_sck_pin=6,
                channel='A',
                gain=64
            )
            hx711.reset()
            measures = hx711.get_raw_data(times=3)
        finally:
            GPIO.cleanup()

        calibrated_measures = [(x - self.OFFSET) / self.SCALE_FACTOR for x in measures]
        return sum(calibrated_measures) / len(calibrated_measures)

    def publish_weight(self):
        weight = self.read_weight()
        msg = Float32()
        msg.data = float(weight)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing weight: {weight:.2f}g')

def main(args=None):
    rclpy.init(args=args)
    load_cell_node = LoadCellNode()
    rclpy.spin(load_cell_node)
    load_cell_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
