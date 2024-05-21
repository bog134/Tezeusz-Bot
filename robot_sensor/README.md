# robot_sensor package

This package provides functionalities for interfacing with robot sensors, specifically lidar sensors, in ROS2. It processes raw sensor data and publishes it in a format suitable for other ROS2 nodes, such as navigation and control nodes.

## Features

* **Lidar Data Processing:** 
    * Subscribes to raw lidar scan data from a specified topic.
    * Extracts relevant information from the lidar data, such as ranges and angles.
    * Publishes processed lidar data as a Float32MultiArray message on a designated topic.
* **Transformations:**
    * Utilizes tf2 to handle coordinate transformations between the robot's base frame and the lidar sensor frame.
    * Ensures that lidar data is published in the correct reference frame for downstream nodes.

## Dependencies

* `rclcpp`: ROS client library for C++.
* `std_msgs`: Provides standard ROS messages, including `Float32MultiArray`.
* `sensor_msgs`:  Provides messages for sensor data, including `LaserScan`.
* `tf2_ros`: Library for handling coordinate transformations in ROS2.


## Example

1. Launch the robot_sensor node
```bash
ros2 launch robot_sensor robot_sensor.launch.py
```

2. In another terminal, echo the processed lidar data
```bash
ros2 topic echo /laser_data
```
