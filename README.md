# wearable_robot_upper_limb

## Required libraries

```
sudo apt-get update && sudo apt-get install python3-pip -y && python3 -m pip install rpi-lgpio && python3 -m pip install hx711
```

## Required ROS packages
- DynamixelSDK  
- dynamixel-workbench 

## Install and build

```sh
mkdir -p ~/dynamixel_ws/src
cd ~/dynamixel_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK
git checkout ros2
```

```sh
cd ~/dynamixel_ws/src
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git dynamixel-workbench
git checkout ros2
cd ..
```

```sh
cd ~/dynamixel_ws/src
git clone https://github.com/donghee/wearable_robot_upper_limb.git
cd ~/dynamixel_ws/src

colcon build
```

## Running

Dynamixel motor node

```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
ros2 run dynamixel_sdk_examples read_write_node
```

HX711 load cell node

```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wearable_robot_upper_limb load_cell
```

Upper limb robot control node

```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wearable_robot_upper_limb robot_control
```
