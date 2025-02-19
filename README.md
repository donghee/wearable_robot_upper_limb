# wearable_robot_upper_limb

The wearable_robot_upper_limb package is designed to control an upper limb wearable robot. This package integrates with Dynamixel motors and HX711 load cell sensors to provide control and feedback for the wearable robot. It is built to work with ROS 2 (humble) and provides various nodes for motor control and sensor data acquisition for wearable robot evaluation.
 
[![Watch the video](https://img.youtube.com/vi/yjoTfqtlwwc/0.jpg)](https://www.youtube.com/watch?v=yjoTfqtlwwc)


## Install and build with Docker

Login to the terminal on the Raspberry Pi 

1. Build docker image
```sh
cd ~/dynamixel_ws
src/wearable_robot_upper_limb/docker/build.sh
```

2. Run docker container
```sh
cd ~/dynamixel_ws
src/wearable_robot_upper_limb/docker/run.sh
```

3. Connect container
```sh
docker attach wearable
docker exec -it wearable bash
```

## Install and build without Docker

1. Install Required tool and libraries
```sh
sudo apt-get install python3-pip curl
python3 -m pip install rpi-lgpio  hx711
```

2. DynamixelSDK for ROS
```sh
mkdir -p ~/dynamixel_ws/src
cd ~/dynamixel_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK
git checkout ros2
```

3. DynamixelSDK for python

```sh
curl -Lo /tmp/3.8.1.tar.gz https://github.com/ROBOTIS-GIT/DynamixelSDK/archive/refs/tags/3.8.1.tar.gz \
    && cd /tmp && tar xvfz 3.8.1.tar.gz  \
    && cd /tmp/DynamixelSDK-3.8.1/python \
    && python3 setup.py install
```

4. Dynamixel workbench for ROS
```sh
cd ~/dynamixel_ws/src
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
cd dynamixel-workbench
git checkout ros2
```

5. Wearable robot upper limb messages package
```sh
cd ~/dynamixel_ws/src
git clone https://github.com/donghee/wearable_robot_upper_limb_msgs.git
```

6. Wearable robot upper limb package
```sh
cd ~/dynamixel_ws/src
git clone https://github.com/donghee/wearable_robot_upper_limb.git
```

7. Build with colcon
```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
```

## Running

Launch wearable upper limb nodes
```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wearable_robot_upper_limb robot_control robot.launch.py
```

## Test (optional)

Test Dynamixel motor node
```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dynamixel_sdk_examples read_write_node
```

Test HX711 load cell node
```sh
cd ~/dynamixel_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wearable_robot_upper_limb load_cell
```
