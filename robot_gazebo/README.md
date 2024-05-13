# robot_gazebo package

This package provides functionalities for setting up and running robot simulations in Gazebo. It includes tools for generating mazes, spawning robots, and configuring simulation environments. 

## Features

* **Maze Generation:** 
    * Generates SDF models of mazes from images using OpenCV.
    * Allows for interactive drawing of maze walls using a graphical interface.
* **Robot Spawning:**
    * Spawns robot models in Gazebo with specified initial positions and orientations. 
* **World Configuration:**
    * Provides launch files for launching Gazebo simulations with different maze environments.

## Dependencies

* `rclcpp`: ROS client library for C++.
* `std_msgs`: Provides standard ROS messages. 
* `gazebo_msgs`: Provides messages and services for interacting with Gazebo. 
* `ament_index_cpp`: Provides tools for accessing ROS package information. 
* `opencv-python`: Provides a set of functions and algorithms for computer vision.
* `numpy`:  Provides tools for working with arrays, matrices, and mathematical functions.

## Example

1. Launch a Gazebo simulation:
    * Train maze:
    ```bash
    ros2 launch robot_gazebo gazebo_world_train.launch.py
    ```
    * Test maze:
    ```bash
    ros2 launch robot_gazebo gazebo_world_test.launch.py
    ```



