import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the entire robot simulation, including Gazebo environment, 
    robot drivers, sensors, navigation, and teleoperation.
    """

    def GiveIncludeLaunchDescription(pkg_name, file_name, condition = None,**kwargs):
        """
        Includes a launch file from a package and allows passing arguments.

        Args:
            pkg_name: The name of the package containing the launch file.
            file_name: The name of the launch file (without the .launch.py extension).
            **kwargs: Keyword arguments to be passed to the included launch file. 

        Returns:
            An IncludeLaunchDescription action. 
        """
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(pkg_name), '/launch', f'/{file_name}.launch.py'
            ]),
            launch_arguments=kwargs.items(),
            condition = condition,
        )
    
    # Declare launch argument to choose between training and testing modes
    world_arg = DeclareLaunchArgument('world_file', default_value='maze_train',
                                     description='world_file: maze_train or maze_test')

    # Condition to check if the selected world is 'maze_train.world'
    condition_train_world = LaunchConfigurationEquals('world_file', 'maze_train')
    condition_test_world = LaunchConfigurationEquals('world_file', 'maze_test')

    # Declare launch argument to choose between training and testing modes
    mode_arg = DeclareLaunchArgument('mode', default_value='None',
                                     description='Mode: None, train,test')
    
    # Condition to check if the selected mode is 'test'
    condition_train_mode = LaunchConfigurationEquals('mode', 'train')
    condition_test_mode = LaunchConfigurationEquals('mode', 'test')

    # Include launch files with arguments
    driver_launch = GiveIncludeLaunchDescription("robot_driver", "robot_driver")
    sensor_launch = GiveIncludeLaunchDescription("robot_sensor", "robot_sensor")  
    train_world_launch = GiveIncludeLaunchDescription("robot_gazebo", "gazebo_world_train", condition_train_world)
    test_world_launch = GiveIncludeLaunchDescription("robot_gazebo", "gazebo_world_test", condition_test_world)    
    train_launch = GiveIncludeLaunchDescription("robot_navigation", "navigation", condition_train_mode, mode='train')
    test_launch = GiveIncludeLaunchDescription("robot_navigation", "navigation", condition_test_mode, mode='test')
    teleop_launch = GiveIncludeLaunchDescription("robot_teleop", "teleop",condition_train_mode)

    # Create and return the launch description
    return LaunchDescription([
        world_arg,  # Condition for selecting the world file
        mode_arg,         # Launch argument for selecting mode
        train_world_launch,     # Launch Gazebo train world
        test_world_launch, # Launch Gazebo test world
        driver_launch,    # Launch robot driver
        sensor_launch,    # Launch sensor processing node
        train_launch, # Launch navigation node in train mode
        test_launch, # Launch navigation node in test mode
        teleop_launch      # Launch teleoperation node 
    ])