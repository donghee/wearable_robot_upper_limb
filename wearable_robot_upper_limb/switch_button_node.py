import rclpy
from rclpy.node import Node
# import gpio status message 

# you need to install rpi-lgpio in raspberry pi 5
#sudo apt remove python3-rpi.gpio 
#sudo apt-get install python3-pip
#pip install rpi-lgpio
import RPi.GPIO as GPIO
import time
import threading

class UpperLimbCommanderNode(Node):
    def __init__(self):
        super().__init__('upper_limb_commander_node')

        self.button_states = [False] * 3
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()

        self.get_logger().info("Initializing gpio button.")

        self.read_thread = threading.Thread(target=self.read_switch_button_loop)
        self.read_thread.start()

        self.timer = self.create_timer(0.01, self.publish_button_states)  # Publish every 0.01 second; 100Hz
        
    def read_switch_button_loop(self):
        button_pins = [5, 6, 13] # RPI GPIO pins
        button_states = [False] * len(buttons)

        while not self.stop_thread.is_set(): 
            try:
                with self.lock:
                    button_states = [GPIO.input(pin) for pin in button_pins]
                    self.button_states = button_states
            except Exception as e:
                self.get_logger().error(e)
            finally:
                continue
                #  print('cleanup')
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    cmd_node = UpperLimbCommanderNode()
    try:
        rclpy.spin(cmd_node)
    except KeyboardInterrupt:
        cmd_node.stop_thread.set()
        cmd_node.read_thread.join()
        cmd_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
