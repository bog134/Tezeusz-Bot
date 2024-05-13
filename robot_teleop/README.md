# robot_teleop package

This package provides a teleoperation interface for controlling a robot using keyboard input. It allows users to send movement commands to the robot.

## Features
* **Keyboard Control:**
    * Uses the `pynput` library to capture keyboard input.
    * Maps specific keys to different movement commands:
        * `w`: robot drives forward
        * `s`: robot drives backward
        * `a`: robot makes one left-turn 
        * `d`: robot makes one right-turn 
        * `left`: robot turns 90-degrees left
        * `right`: robot turns 90-degrees right
        * `1`: robot drives straight until it's 0.35 away form a wall (it is capable of performing a 180-degree turn when facing a wall)
        * `2`: robot drives straight until it's 0.2 away form a wall (it is capable of performing a 90-degree turn when facing a wall)
        * `3`: robot halts at a distance from the edge of the turn sufficient to allow for a subsequent 180-degree rotation away from the wall
        * `4`: robot halts at a distance from the edge of the turn sufficient to allow for a subsequent 90-degree rotation away from the wall
* **Lidar and Odometry Integration:**
    * Subscribes to lidar data to detect obstacles and adjust movements accordingly.
    * Uses odometry data to track the robot's position and orientation.
    * Allows for more informed and responsive teleoperation based on sensor feedback.

## Dependencies
* `rclcpp`: ROS client library for C++.
* `std_msgs`: Provides standard ROS messages, including String for sending movement commands.
* `nav_msgs`: Provides messages for navigation, including Odometry for robot pose information.
* `pynput`: Library for handling keyboard input in Python.
* `scipy`: Scientific computing library used for rotation calculations.
* `numpy`: Library for numerical computations.
 
## Example

1. Launch the robot_teleop node
```bash
ros2 launch robot_teleop teleop.launch.py
```

2. In another terminal, echo the processed movement commands
```bash
ros2 topic echo /move_data
```
