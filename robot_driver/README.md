# robot_driver package

## Description

This package provides a ROS2 node for controlling the movement of a robot. It translates high-level movement commands (e.g., forward, backward, left, right) into appropriate linear and angular velocities for the robot's base controller.

## Dependencies
- `rclcpp`: ROS client library for C++;
- `geometry_msgs`: Provides messages for geometry data, including Twist messages for velocity commands;
- `std_msgs`: Provides standard ROS messages, including String messages for movement commands.

## Examples
1. Launch the robot_driver node:
```
ros2 launch robot_driver robot_driver.launch.py
```
2. Publish movement commands:
Publish string messages (e.g., "forward", "backward", "left", "right") to the /move_data topic to control the robot's movement.
```
ros2 topic pub /move_data std_msgs/msg/String "data: 'forward'"
```
3. Subscribe velocity commands
```
ros2 topic echo /demo/cmd_vel
```