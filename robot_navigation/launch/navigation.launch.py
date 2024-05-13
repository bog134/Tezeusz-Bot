from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    """Generates launch description for either training or testing the robot navigation model."""

    # Declare launch argument to choose between modes
    mode_arg = DeclareLaunchArgument('mode', default_value='train',
                                     description='Mode: train, test')
    
    # Condition to check if the selected mode is 'test'
    condition_train = LaunchConfigurationEquals('mode', 'train')
    condition_test = LaunchConfigurationEquals('mode', 'test')


    # Return the launch description with the dynamically chosen executable
    return LaunchDescription([
        mode_arg,
        Node(
            package='robot_navigation', 
            executable='train',
            output='screen',
            condition = condition_train),
        Node(
            package='robot_navigation', 
            executable='test',
            output='screen',            
            condition = condition_test),
    ])