
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
            executable='robot_state',
            name='robot_state',
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
            package='wearable_robot_upper_limb',
            executable='robot_command',
            name='robot_command',
            output='screen',
            parameters=[]
        ),
    ])
