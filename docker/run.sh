# delete previous container
docker stop wearable; docker rm wearable

# running new container
#docker run -it --name wearable -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws arm64v8/ros:humble bash -c "sudo apt-get update && sudo apt-get install python3-pip -y && python3 -m pip install rpi-lgpio && python3 -m pip install hx711 && cd /dynamixel_ws/src/wearable_robot_upper_limb/docker/DynamixelSDK-3.8.1/python/ && python3 setup.py install && source /dynamixel_ws/install/setup.bash && ros2 launch wearable_robot_upper_limb robot.launch.py"
docker run -it --name wearable -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws ghcr.io/donghee/wearable_robot_upper_limb:latest bash -c "source /opt/ros/humble/setup.bash && source /dynamixel_ws/install/setup.bash && ros2 launch wearable_robot_upper_limb robot.launch.py"
#docker run -it --name wearable -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws ghcr.io/donghee/wearable_robot_upper_limb:latest 


