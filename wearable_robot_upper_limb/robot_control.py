#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from wearable_robot_upper_limb_msgs.srv import UpperLimbCommand
from geometry_msgs.msg import Vector3
import time
from dynamixel_sdk import *
import threading


# Control table address
# https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/#control-table-description
ADDR_BAUDRATE           = 8
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_VELOCITY      = 104
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_LOAD       = 126
ADDR_PRESENT_VELOCITY   = 128
ADDR_PRESENT_POSITION   = 132
ADDR_PROFILE_VELOCITY   = 112
ADDR_OPERATING_MODE     = 11

OP_CURRENT_BASED_POSITION = 5
OP_VELOCITY = 1

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600, Dynamixel XH430 series
#BAUDRATE                    = 2000000           # Dynamixel XH540 series
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MINIMUM_VELOCITY_VALUE  = 0                 # Minimum velocity value
DXL_MAXIMUM_VELOCITY_VALUE  = 1023              # Maximum velocity value
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
MAX_VALUE_4BYTES = 4294967295  # 2^32 - 1

class DynamixelController:
    def __init__(self, node_logger):
        self.logger = node_logger
        
        # Constants
        self.DXL_ID = DXL_ID
        self.PROTOCOL_VERSION = PROTOCOL_VERSION
        self.BAUDRATE = BAUDRATE
        self.DEVICENAME = DEVICENAME

        # Initialize Dynamixel
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        self._initialize_connection()

    def _initialize_connection(self):
        if self.portHandler.openPort():
            self.logger.info("Succeeded to open port")
        else:
            self.logger.error("Failed to open port")
            return False
            
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.logger.info("Succeeded to change baudrate")
        else:
            self.logger.error("Failed to change baudrate")
            return False
        return True

    def ping(self):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.logger.info("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number))

    def disable_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.logger.info("Disabled torque")

    def enable_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.logger.info("Enabled torque")
        
    def set_operating_mode(self, mode):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.logger.info("Set operating mode")
       
    def set_baudrate(self, baudrate):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_BAUDRATE, baudrate)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write_control_table(self, addr=ADDR_PROFILE_VELOCITY, value=30):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, addr, value)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
 
    def get_present_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if dxl_present_position > DXL_MAXIMUM_POSITION_VALUE:
            dxl_present_position = dxl_present_position - MAX_VALUE_4BYTES

        return dxl_present_position * (360/4096)

    def get_present_velocity(self):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if dxl_present_velocity > DXL_MAXIMUM_VELOCITY_VALUE:
            dxl_present_velocity = dxl_present_velocity - MAX_VALUE_4BYTES

        return dxl_present_velocity

    def set_goal_position(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, 
            int(position * (4096/360))
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_goal_velocity(self, velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_GOAL_VELOCITY, 
            int(velocity)
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
       
    def get_present_current(self):
        current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_PRESENT_LOAD
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.logger.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return current

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()
        self.logger.info("Port closed")

class UpperLimbNode(Node):
    def __init__(self):
        super().__init__('wearable_robot_upper_limb_controller')
        
        # loop time
        self.DELTA_TIME = 0.0125  # 80Hz
        
        # Parameters
        self.declare_parameter('robot_weight', 200.0)  # g
        self.declare_parameter('l1', 0.18)  # m
        self.declare_parameter('l2', 0.30)  # m
        #  self.declare_parameter('repeat', 40)
        self.declare_parameter('repeat', 5)
        self.declare_parameter('delay_time', 1000)  # ms
        
        # Control parameters
        self.m = 0.5
        self.c = 10
        self.k = 0
        self.k2 = 1
        self.a = -90/2
        
        # State variables
        self.current_time = 0.0
        self.direction = -1
        self.flag = 2
        self.r = -1
        self.start = 0
        self.theta = 0.0

        # thread 
        self.is_running = True
        
        # Initialize Dynamixel
        self.previous_velocity = 0.0
       
        # Publishers
        self.angle_pub = self.create_publisher(Float32, 'elbow_angle', 10)
        self.force_pub = self.create_publisher(Float32, 'loadcell_force', 10)
        self.current_pub = self.create_publisher(Float32, 'motor_current', 10)
        self.state_pub = self.create_publisher(Int32, 'device_state', 10)

        # Subscribers
        self.loadcell_value = None
        self.loadcell_sub = self.create_subscription(Float32, 'load_cell_weight', self.loadcell_callback, 10)

        # Services
        self.command_service = self.create_service(UpperLimbCommand, 'upper_limb_command', self.handle_command)
        
        # Timer for control loop (80Hz)
        self.timer = self.create_timer(self.DELTA_TIME, self.control_loop)

        # Motor controller
        self.dxl = DynamixelController(self.get_logger())
        self.reset_motor_position()


    def __del__(self):
        self.stop()
        self.dxl.close()
        self.get_logger().info("__del__ Port closed")

    def reset_motor_position(self):
        # ping
        self.dxl.ping()

        # Disable Dynamixel Torque
        self.dxl.disable_torque()

        # Set operating mode
        self.dxl.set_operating_mode(OP_CURRENT_BASED_POSITION)

        # Enable Dynamixel Torque
        self.dxl.enable_torque()
 
        self.dxl.write_control_table(ADDR_PROFILE_VELOCITY, 30)
        self.dxl.set_goal_position(180.0)

        # Initialize motor position
        current_position = self.dxl.get_present_position()
        while abs(current_position - 180.0) > DXL_MOVING_STATUS_THRESHOLD: # Wait until the 180 goal position is reached
            current_position = self.dxl.get_present_position()
            time.sleep(0.1)

        time.sleep(1.0)
        self.dxl.write_control_table(ADDR_PROFILE_VELOCITY, 60)
        time.sleep(2.0)
        self.get_logger().info("motor reposition is complete")

        # Timer for control loop (80Hz)
        # Disable Dynamixel Torque
        self.dxl.disable_torque()
        # Set operating mode
        self.dxl.set_operating_mode(OP_VELOCITY)
        # Enable Dynamixel Torque
        self.dxl.enable_torque()

    def stop(self):
        self.timer.cancel()
        self.dxl.disable_torque()
        self.get_logger().info('Stopping...')

    def control_loop(self):
        if not self.is_running or self.r > self.get_parameter('repeat').value:
            self.get_logger().info(f'quitting: {self.r}')
            self.dxl.set_goal_velocity(0)
            self.stop()
            return

        # Read sensors
        current_position = self.dxl.get_present_position()
        current_velocity = self.dxl.get_present_velocity() * 6  # 6 From KNU's firmware
        loadcell_value = self.read_loadcell()  # Implement according to your HX711 interface
        current = self.dxl.get_present_current()

        #  self.get_logger().info(f'direction: {self.direction}, velocity: {current_velocity}, positiion: {current_position}, theta: {self.theta}, current: {current}')
       
        # Calculate control
        self.current_time += 12.5
        
        if current_position >= 165 and self.direction == -1:
            self.direction = 1
            self.r += 1
            self.current_time = 0
        elif current_position <= 75 and self.direction == 1:
            self.direction = -1
            self.r += 1
            self.current_time = 0
            
        if self.direction > 0:
            self.theta = self.a * self.current_time / 1000 + 165
        else:
            self.theta = -self.a * self.current_time / 1000 + 75
            
        # Impedance control
        delta_velocity = current_velocity - self.previous_velocity
        delta_velocity = max(min(delta_velocity, 20.0), -20.0)
        acceleration = delta_velocity / self.DELTA_TIME
        #  acceleration = 0
        #  self.get_logger().info(f'acceleration: {acceleration}')
        self.previous_velocity = current_velocity

        delta_force = loadcell_value + 50 - 300
        velocityT = self.calculate_velocity(current_position, acceleration, delta_force, self.theta)
        self.dxl.set_goal_velocity(velocityT / 6) # 6 From KNU's firmware
        #  self.dxl.set_goal_velocity(velocityT)

        self.publish_state(current_position, loadcell_value, current)
        
    def calculate_velocity(self, position, acceleration, force, theta):
        velocity = self.a * self.direction + (force - self.m * acceleration - self.k * (position - theta)) / self.c
        return max(min(velocity, 70.0), -70.0)
        
    def publish_state(self, position, loadcell_value, current):
        angle_msg = Float32()
        angle_msg.data = float(position)
        self.angle_pub.publish(angle_msg)
        
        force_msg = Float32()
        force_msg.data = float(loadcell_value)
        self.force_pub.publish(force_msg)
        
        current_msg = Float32()
        current_msg.data = float(current)
        self.current_pub.publish(current_msg)
        
        state_msg = Int32()
        state_msg.data = self.flag

        # Publish data
        # State : Flexion , Repeat : 6 , Weight : 475.40g , Elbow Angle : 125.22° , Current : -304.0000mA
        # State : Extension , Repeat : 5 , Weight : -92.22g , Elbow Angle : 151.98° , Current : 46.0000mA
        state = 'Flexion' if self.direction == 1 else 'Extension'
        self.get_logger().info(f'State : {state}, Repeat : {self.r}, Weight : {loadcell_value:.2f}g, Elbow Angle : {position:.2f}°, Current : {current:.4f}mA')

        self.state_pub.publish(state_msg)
        
    def loadcell_callback(self, msg):
        self.loadcell_value = msg.data

    def read_loadcell(self):
        if self.loadcell_value is not None:
            return self.loadcell_value
        return 0.0  # TODO need error handling

    def handle_command(self, request, response):
        print(f"Received command: Task {request.task} and {request.command}")
        response.success = True
        response.message = "Command received"
        return response

def disable_torque():
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if portHandler.openPort():
        print("Succeeded to open port")
    else:
        print("Failed to open port")
        return False
        
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change baudrate")
    else:
        print("Failed to change baudrate")
        return False

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Disabled torque")
    portHandler.closePort()

    return dxl_error == 0
        
def main(args=None):
    rclpy.init(args=args)
    upper_limb_node = UpperLimbNode()

    try:
        rclpy.spin(upper_limb_node)
    except KeyboardInterrupt:
        upper_limb_node.get_logger().error('Keyboard Interrupt received')
    finally:
        upper_limb_node.get_logger().error('Finally')

        while not disable_torque(): # Retry until torque is disabled
            upper_limb_node.get_logger().error("Failed to disable torque, retrying...")
        rclpy.try_shutdown()

    disable_torque() # One more disable torque before exiting
    upper_limb_node.get_logger().info("Torque disabled!")

if __name__ == '__main__':
    main()
