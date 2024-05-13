import os
from launch import LaunchDescription
from launch_ros.actions import Node 
 
def generate_launch_description():
  """Generates launch description for the robot_sensor package."""

  return LaunchDescription([
    # Launch the robot_sensor node
    Node(
      package='robot_sensor', 
      executable='robot_sensor',
      output='screen'),

    # Launch static transform publisher
    Node(
      package = 'tf2_ros',
      executable = 'static_transform_publisher',
      output='screen',
      # Arguments define the static transform from base_link to laser_link
      arguments = ["0.01", "0", "0.0175", "0.0", "0.0", "0.0", "-1.0", "base_link", "laser_link"]
    )
  ])