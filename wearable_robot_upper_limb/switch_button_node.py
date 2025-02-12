import rclpy
from rclpy.node import Node
# import gpio status message 
from std_msgs.msg import BoolMultiArray

# you need to install rpi-lgpio in raspberry pi 5
#sudo apt remove python3-rpi.gpio 
#sudo apt-get install python3-pip
#pip install rpi-lgpio
import RPi.GPIO as GPIO
import time
import threading

class SwitchButtonNode(Node):
    def __init__(self):
        super().__init__('switch_button_node')

        self.button_states = [False] * 3
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()

        self.get_logger().info("Initializing gpio button.")

        self.read_thread = threading.Thread(target=self.read_switch_button_loop)
        self.read_thread.start()

        # publish multiple button states as one message
        self.button_states_pub = self.create_publisher(BoolMultiArray, 'button_states', 10)
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

    def publish_button_states(self):
        msg = BoolMultiArray()
        msg.data = self.button_states
        self.button_states_pub.publish(msg)
        self.get_logger().info(f'Publishing button states: {self.button_states}')

def main(args=None):
    rclpy.init(args=args)
    button_node = SwitchButtonNode()
    try:
        rclpy.spin(button_node)
    except KeyboardInterrupt:
        button_node.stop_thread.set()
        button_node.read_thread.join()
        button_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
