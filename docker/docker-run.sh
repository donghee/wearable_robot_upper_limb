docker run -it --name wearable -d --restart unless-stopped --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v=/dev:/dev -v=$HOME/dynamixel_ws:/dynamixel_ws arm64v8/ros:humble
