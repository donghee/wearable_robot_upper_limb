# stop and delete previous container
docker stop wearable; docker rm wearable

# If the machine is x86_64, run the container with arm64 platform
if [ "$(uname -m)" = "x86_64" ]; then
  docker run -it --name wearable --platform linux/arm64 -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/tmp/dynamixel_ws:/dynamixel_ws ghcr.io/donghee/wearable_robot_upper_limb:latest
  exit 0
fi

# running new container in raspberry pi 5
## auto start robot.launch.py during boot.
docker run -it --name wearable -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws ghcr.io/donghee/wearable_robot_upper_limb:latest bash -c "source /opt/ros/humble/setup.bash && source /dynamixel_ws/install/setup.bash && ros2 launch wearable_robot_upper_limb robot.launch.py"

## manual start
# docker run -it --name wearable --rm --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws ghcr.io/donghee/wearable_robot_upper_limb:latest 
