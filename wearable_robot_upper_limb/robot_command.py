import rclpy
from rclpy.node import Node

# you need to install rpi-lgpio in raspberry pi 5
#sudo apt remove python3-rpi.gpio 
#sudo apt-get install python3-pip
#pip install rpi-lgpio
import RPi.GPIO as GPIO
import time
import threading
from wearable_robot_upper_limb_msgs.srv import UpperLimbCommand

# RPi.GPIO BCM pin number
START_BUTTON_PIN = 23
RESET_BUTTON_PIN = 24
TASK3_BUTTON_PIN = 25
TASK1_BUTTON_PIN = 8

class UpperLimbCommanderNode(Node):
    def __init__(self):
        super().__init__('upper_limb_commander_node')
        self.button_states = {START_BUTTON_PIN: False, RESET_BUTTON_PIN: False, TASK3_BUTTON_PIN: False, TASK1_BUTTON_PIN: False}

        self.get_logger().info("Initializing gpio button.")

        # Service
        self.selected_task = 2
        self.upper_limb_command_client = self.create_client(UpperLimbCommand, 'upper_limb_command')
        
        while not self.upper_limb_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service upper_limb_command...')

        # Read button states in a separate thread
        self.lock = threading.Lock()
        self.stop_thread = threading.Event()
        self.read_thread = threading.Thread(target=self.read_buttons_loop)
        self.read_thread.start()

        # TODO: delete this timer
        self.timer = self.create_timer(0.1, self.publish_button_states)  # Publish every 0.1 second; 10Hz

        # Call reset command when the node is started
        #self.upper_limb_command_client.call_async(UpperLimbCommand.Request(task=self.selected_task, command=UpperLimbCommand.Request.COMMAND_RESET)) # Reset command
        self.get_logger().info(f"Task {self.selected_task} and Reset command is called.")

    def read_buttons_loop(self):
        GPIO.setmode(GPIO.BCM)
        for pin in self.button_states.keys():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        new_button_states = self.button_states.copy()

        while not self.stop_thread.is_set(): 
            try:
                with self.lock:
                    for pin in self.button_states.keys():
                        new_button_states[pin] = not GPIO.input(pin)
                    self.handle_button_states(new_button_states)
            except Exception as e:
                self.get_logger().error(e)
            finally:
                time.sleep(0.5)  # sleep 500ms for button debouncing
                continue
        GPIO.cleanup()

    def handle_button_states(self, new_button_states):
        prev_button_states = self.button_states.copy()
        self.button_states = new_button_states.copy()

        if not prev_button_states[START_BUTTON_PIN] and new_button_states[START_BUTTON_PIN]:
            self.get_logger().info(f"Task {self.selected_task} and Start command is called.")
            response = self.upper_limb_command_client.call_async(UpperLimbCommand.Request(task=self.selected_task, command=UpperLimbCommand.Request.COMMAND_START_STOP))
        if self.button_states[RESET_BUTTON_PIN] and not prev_button_states[RESET_BUTTON_PIN]:
            self.get_logger().info(f"Task {self.selected_task} and Reset command is called.")
            response = self.upper_limb_command_client.call_async(UpperLimbCommand.Request(task=self.selected_task, command=UpperLimbCommand.Request.COMMAND_RESET))

        if not self.button_states[TASK1_BUTTON_PIN] and self.button_states[TASK3_BUTTON_PIN]:
            #  self.get_logger().info("Task 3 selected")
            self.selected_task = UpperLimbCommand.Request.TASK_3
        if not self.button_states[TASK1_BUTTON_PIN] and not self.button_states[TASK3_BUTTON_PIN]:
            #  self.get_logger().info("Task 2 selected")
            self.selected_task = UpperLimbCommand.Request.TASK_2
        if self.button_states[TASK1_BUTTON_PIN] and not self.button_states[TASK3_BUTTON_PIN]:
            #  self.get_logger().info("Task 1 selected")
            self.selected_task = UpperLimbCommand.Request.TASK_1

    def publish_button_states(self):
        #  self.get_logger().info(f"Publishing button states: {self.button_states}")
        pass

    def upper_limb_command_callback(self, request, response):
        self.get_logger().info(f"Received command: {request.command} for task: {request.task}")
        self.get_logger().info(f"Response: {response.success}, {response.message}")
        return response

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
