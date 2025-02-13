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
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
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


class UpperLimbNode(Node):
    def __init__(self):
        super().__init__('wearable_robot_upper_limb_controller')
        
        # Constants
        self.DXL_ID = DXL_ID
        self.PROTOCOL_VERSION = PROTOCOL_VERSION
        self.BAUDRATE = 57600 # Dynamixel XH430 series
        #self.BAUDRATE = 2000000 # Dynamixel XH540 series
        self.DEVICENAME = DEVICENAME

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

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open port")
        else:
            self.get_logger().error("Failed to open port")
            return
            
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info("Succeeded to change baudrate")
        else:
            self.get_logger().error("Failed to change baudrate")
            return

        # set baudrate
        #  self.set_baudrate_(9)  # 2000000
        #  self.set_baudrate_(34) # 57600

        # ping
        self.ping_()

        # Disable Dynamixel Torque
        self.disable_torque_()

        # Set operating mode
        self.set_operating_mode_(OP_CURRENT_BASED_POSITION)

        # Enable Dynamixel Torque
        self.enable_torque_()
        
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
        
        # Initialize position
        self.write_control_table_(ADDR_PROFILE_VELOCITY, 30)
        self.set_goal_position(180.0)

        current_position = self.get_present_position()
        while abs(current_position - 180.0) > DXL_MOVING_STATUS_THRESHOLD: # Wait until the 180 goal position is reached
            current_position = self.get_present_position()
            time.sleep(0.1)

        time.sleep(1.0)
        self.write_control_table_(ADDR_PROFILE_VELOCITY, 60)
        time.sleep(2.0)
        self.get_logger().info("Initialization complete")

        # Timer for control loop (80Hz)
        # Disable Dynamixel Torque
        self.disable_torque_()
        # Set operating mode
        self.set_operating_mode_(OP_VELOCITY)
        # Enable Dynamixel Torque
        self.enable_torque_()

        self.timer = self.create_timer(self.DELTA_TIME, self.control_loop)

    def __del__(self):
        self.stop()
        self.portHandler.closePort()
        self.get_logger().info("__del__ Port closed")

    def stop(self):
        self.timer.cancel()
        self.disable_torque_()
        self.get_logger().info('Stopping...')

    def control_loop(self):
        if not self.is_running or self.r >= self.get_parameter('repeat').value:
            self.get_logger().info(f'quitting: {self.r}')
            self.set_goal_velocity(0)
            self.stop()
            return

        # Read sensors
        current_position = self.get_present_position()
        current_velocity = self.get_present_velocity() * 6  # 6 From KNU's firmware
        loadcell_value = self.read_loadcell()  # Implement according to your HX711 interface
        current = self.get_present_current()

        self.get_logger().info(f'direction: {self.direction}, velocity: {current_velocity}, positiion: {current_position}, theta: {self.theta}')
        
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
        self.get_logger().info(f'acceleration: {acceleration}')
        self.previous_velocity = current_velocity

        delta_force = loadcell_value + 50 - 300
        velocityT = self.calculate_velocity(current_position, acceleration, delta_force, self.theta)
        self.set_goal_velocity(velocityT / 6) # 6 From KNU's firmware
        #  self.set_goal_velocity(velocityT)

        # Publish data
        self.publish_state(current_position, loadcell_value, current)
        
    def calculate_velocity(self, position, acceleration, force, theta):
        velocity = self.a * self.direction + (force - self.m * acceleration - self.k * (position - theta)) / self.c
        return max(min(velocity, 70.0), -70.0)
        
    def publish_state(self, position, force, current):
        angle_msg = Float32()
        angle_msg.data = float(position)
        self.angle_pub.publish(angle_msg)
        
        force_msg = Float32()
        force_msg.data = float(force)
        self.force_pub.publish(force_msg)
        
        current_msg = Float32()
        current_msg.data = float(current)
        self.current_pub.publish(current_msg)
        
        state_msg = Int32()
        state_msg.data = self.flag
        self.state_pub.publish(state_msg)
        
    # Dynamixel helper functions
    def ping_(self):
        # Get Dynamixel model number
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            #  print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            #  print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            #  print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number))
            self.get_logger().info("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number))

    def disable_torque_(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            #  print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            #  print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            #  print("Dynamixel has been successfully connected")
            self.get_logger().info("Disabled torque")

    def enable_torque_(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            #  print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            #  print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info("Enabled torque")
        
    def set_operating_mode_(self, mode):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info("Set operatiing mode")
       
    def set_baudrate_(self, baudrate):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_BAUDRATE, baudrate)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write_control_table_(self, addr=ADDR_PROFILE_VELOCITY, value=30):
        # Set profile velocity
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, addr, value)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
 
    def get_present_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
        #  self.get_logger().info("Present Position of ID %s = %s" % (self.DXL_ID, dxl_present_position))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #  else:
        #      self.get_logger().info("Get present position")

        if dxl_present_position > DXL_MAXIMUM_POSITION_VALUE:
            dxl_present_position = dxl_present_position - MAX_VALUE_4BYTES # 4294967295

        return dxl_present_position * (360/4096)

    def get_present_velocity(self):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #  else:
        #      self.get_logger().info("Get present velocity")

        if dxl_present_velocity > DXL_MAXIMUM_VELOCITY_VALUE:
            dxl_present_velocity = dxl_present_velocity - MAX_VALUE_4BYTES # 4294967295 

        return dxl_present_velocity


    def set_goal_position(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx( self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, int(position * (4096/360)))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #  else:
        #      self.get_logger().info("Set goal position")

    def set_goal_velocity(self, velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_GOAL_VELOCITY, 
            int(velocity)
            #  int(velocity * (6 * 0.229))
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #  else:
        #      self.get_logger().info("Set goal velocity")
       
    def get_present_current(self):
        current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx( self.portHandler, self.DXL_ID, ADDR_PRESENT_LOAD)
        #  self.get_logger().info("Present Current of ID %s = %s" % (self.DXL_ID, current))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #  else:
        #      self.get_logger().info("Get present current")
        return current
        
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
        return
        
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change baudrate")
    else:
        print("Failed to change baudrate")
        return

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Disabled torque")

    portHandler.closePort()
        
def main(args=None):
    rclpy.init(args=args)
    upper_limb_node = UpperLimbNode()

    try:
        rclpy.spin(upper_limb_node)
    except KeyboardInterrupt:
        upper_limb_node.get_logger().error('Keyboard Interrupt received')
    finally:
        upper_limb_node.get_logger().error('Finally')
        upper_limb_node.portHandler.closePort()
        rclpy.try_shutdown()
        disable_torque()

if __name__ == '__main__':
    main()
