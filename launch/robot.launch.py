
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wearable_robot_upper_limb',
            executable='robot_control',
            name='robot_control',
            output='screen',
            parameters=[]
        ), 
        Node(
            package='wearable_robot_upper_limb',
            executable='load_cell',
            name='load_cell',
            output='screen',
            parameters=[]
        ), 
        Node(
            package='dynamixel_sdk_examples',
            executable='read_write_node',
            name='read_write_node',
            output='screen',
            parameters=[]
        ),
    ])
