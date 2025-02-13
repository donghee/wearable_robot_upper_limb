
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_sdk_examples',
            executable='read_write_node',
            name='read_write_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='wearable_robot_upper_limb',
            executable='dynamixel_pub',
            name='dynamixel_pub',
            output='screen',
            parameters=[]
        )
    ])
