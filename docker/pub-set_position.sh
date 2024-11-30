source /opt/ros/humble/setup.bash
source /dynamixel_ws/install/setup.bash

sleep 2
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{ id: 1, position: 1000}"
sleep 2
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{ id: 1, position: 500}"
sleep 2
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{ id: 1, position: 1000}"
sleep 2
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{ id: 1, position: 0}"

