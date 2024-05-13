from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 
    return LaunchDescription([
        Node(
            package='robot_teleop', 
            executable='teleop',
            output='screen')
    ])