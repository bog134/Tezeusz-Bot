# robot_navigation package

This package provides functionalities for robot navigation within an environment. It uses sensor data, such as lidar scans, along with a trained neural network (AlexNet) 
to make navigation decisions and guide the robot towards its goal.

## Features
* **Sensor Fusion:**
    * Uses data from lidar sensors to create a comprehensive understanding of the robot's surroundings.
* **Neural Network-based Navigation:**
    * Utilizes a pre-trained AlexNet model to process sensor data and extract relevant features for navigation.
    * The network can be fine-tuned or retrained to improve performance in specific environments or tasks.
    * Based on the network's output, the package determines appropriate navigation commands, such as turning, moving forward, or stopping.
* **Path Planning and Obstacle Avoidance:**
    * Implements algorithms for path planning and obstacle avoidance to ensure the robot reaches its destination safely and efficiently.

## Dependencies
* `rclcpp`: ROS client library for C++.
* `sensor_msgs`: Provides messages for sensor data, including LaserScan and Odometry.
* `geometry_msgs`: Provides messages for geometric primitives, such as poses and transforms.
* `nav_msgs`: Provides messages for navigation, such as Path and OccupancyGrid.

* `opencv-python`: Provides a set of functions and algorithms for computer vision.
* `numpy`:  Provides tools for working with arrays, matrices, and mathematical functions.
* `Tensorflow`: Provides an ecosystem for developing and deploying machine learning models.
* `scikit-learn`: Uses for splitting train data

## Example

1. Collect training data:
```bash
ros2 launch robot_navigation navigation.launch.py mode:=train
```
2. Train model:
```bash
python3 src/robot_navigation/robot_navigation/model/main.py
```
2. Test model in Gazebo environment:
```bash
ros2 launch robot_navigation navigation.launch.py mode:=test
```