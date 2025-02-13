import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# hx711 library use the old GPIO code, for using hx711 library, you need to install rpi-lgpio in raspberry pi 5
#sudo apt remove python3-rpi.gpio 
#sudo apt-get install python3-pip
#pip install rpi-lgpio
#pip install hx711
import RPi.GPIO as GPIO
from hx711 import HX711
import time
import threading

class LoadCellNode(Node):
    def __init__(self):
        super().__init__('load_cell_node')

        self.hx711 = None
        self.OFFSET = 0
        self.SCALE_FACTOR = 279  # Get scale factor from calibration; scale factor 279 borrow from KNU's upper limb arduino firmware
        self.current_weight = None
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()

        self.get_logger().info("Initializing load cell...")
        self.init_calibration()
        self.get_logger().info("Load cell initialized.")

        self.read_thread = threading.Thread(target=self.read_weight_loop)
        self.read_thread.start()

        self.publisher_ = self.create_publisher(Float32, 'load_cell_weight', 10)
        self.timer = self.create_timer(0.01, self.publish_weight)  # Publish every 0.01 second; 100Hz
        
    def init_calibration(self):
        # Calibration
        try:
            self.hx711 = HX711(
                dout_pin=5,
                pd_sck_pin=6,
                channel='A',
                gain=128
            )
            self.hx711.reset()
            measures = self.hx711.get_raw_data(times=10)
            self.OFFSET = sum(measures) / len(measures)
        finally:
            GPIO.cleanup()

    def read_weight_loop(self):
        self.hx711 = HX711(
            dout_pin=5,
            pd_sck_pin=6,
            channel='A',
            gain=128
        )
        self.hx711.reset()

        while not self.stop_thread.is_set(): 
            try:
                measures = self.hx711.get_raw_data(times=3)
                calibrated_measures = [(x - self.OFFSET) / self.SCALE_FACTOR for x in measures]
                with self.lock:
                    self.current_weight = sum(calibrated_measures) / len(calibrated_measures)
            except Exception as e:
                self.get_logger().error(e)
            finally:
                continue
                #  print('cleanup')
        GPIO.cleanup()

    def publish_weight(self):
        if self.current_weight is None:
            return
        msg = Float32()
        msg.data = float(self.current_weight)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing weight: {self.current_weight:.2f}g')

def main(args=None):
    rclpy.init(args=args)
    load_cell_node = LoadCellNode()
    try:
        rclpy.spin(load_cell_node)
    except KeyboardInterrupt:
        load_cell_node.stop_thread.set()
        load_cell_node.read_thread.join()
        load_cell_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
