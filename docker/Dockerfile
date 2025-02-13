FROM arm64v8/ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install \
    rpi-lgpio \
    hx711

RUN curl -Lo /tmp/3.8.1.tar.gz https://github.com/ROBOTIS-GIT/DynamixelSDK/archive/refs/tags/3.8.1.tar.gz \
    && cd /tmp && tar xvfz 3.8.1.tar.gz  \
    && cd /tmp/DynamixelSDK-3.8.1/python \
    && python3 setup.py install

WORKDIR /dynamixel_ws

# 소스 코드 복사
#COPY . /dynamixel_ws/src/wearable_robot_upper_limb/

# ROS2 워크스페이스 설정
#RUN /bin/bash -c '. /opt/ros/humble/setup.bash && \
#    colcon build --symlink-install'

# 환경 변수 설정
#ENV ROS_DOMAIN_ID=0
#ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# entrypoint 스크립트 추가
COPY ./docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# 기본 실행 명령
CMD ["bash"]
