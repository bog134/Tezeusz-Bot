import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
 
def generate_launch_description():
    """
    Launch Gazebo simulation with the train maze.
    """
 
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    # Set the path to this package.
    pkg_share = FindPackageShare(package='robot_gazebo').find('robot_gazebo')

    # Set the path to the world file
    world_file_name = 'maze_train.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

    # Specify the actions

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # Akcja spawnująca jednostkę w symulatorze Gazebo
    spawn_entity = Node(
        package='robot_gazebo', 
        executable='robot_gazebo',
        arguments=['TezeuszBot', 'demo', '2.16', '-4.95', '0.0', '0.7071068', '0.0', '0.0', '0.7071068'],
        output='screen'
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity)

    return ld
