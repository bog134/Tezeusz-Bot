# Tezeusz-Bot

This project simulates a robot navigating through a maze environment using ROS2 and Gazebo. The robot uses lidar sensor data and odometry along with a trained neural network (AlexNet) to make navigation decisions and reach its goals. The project also includes teleoperation capabilities for manual control of the robot using a keyboard.

## Table of Contents
1. [Project Structure](#project-structure)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Getting Started](#getting-started)
5. [Additional Notes](#additional-notes)


## Project Structure
The project is organized into several ROS2 packages, each with specific functionalities:
* **`robot_description`:** Contains the robot's model and description files.
* **`robot_driver`:** Implements the robot's motor driver for controlling movement.
* **`robot_gazebo`:** Provides tools for setting up and running the Gazebo simulation environment, including maze generation and robot spawning.
* **`robot_launch`:** Contains launch files for starting and configuring the various components of the robot system.
* **`robot_navigation`:** Implements the robot's navigation system, which uses sensor data and a neural network for path planning and obstacle avoidance.
* **`robot_sensor`:** Processes raw sensor data (e.g., lidar scans) and publishes it in a format suitable for other nodes.
* **`robot_teleop`:** Enables teleoperation of the robot using keyboard input.

## Features
* **Gazebo Simulation:** The project utilizes Gazebo to create a realistic simulation environment for the robot, including different maze configurations.
* **Lidar-based Navigation:** The robot uses lidar sensor data to perceive its surroundings and make navigation decisions.
* **Neural Network Integration:** A pre-trained AlexNet model is used to process sensor data and extract features for navigation.
* **Teleoperation:** The robot can be manually controlled using keyboard commands for testing and exploration.
* **Modular Design:** The project is organized into separate ROS2 packages, promoting modularity and reusability.

## Dependencies
* Python 3.10
* ROS2 Humble
* Ubuntu 20.04
* tensorflow
* scikit-learn
* opencv-python
* numpy

## Getting Started
1. **Create a ROS2 workspace** 
2. **Clone the repository:**
```bash
git clone https://github.com/bog134/Tezeusz-Bot.git
```
3. **Copy files and folders from Tezeusz-Bot folder to `src` folder** 
3. **Install necessary dependencies**
```bash
pip install -r src/requirements.txt
```
3. **Build the project**
4. **Source the overlay**
5. **Append path to python packages (if necessary)**
6. **Launch the training simulation**
```bash
ros2 launch robot_launch simulation.launch.py world_file:=maze_train mode:=train
```
7. **Drive the robot through the maze and close the simulation**
8. **Train model**
```bash
python3 src/robot_navigation/robot_navigation/model/main.py
```
9. **Launch the testing simulation:**
```bash
ros2 launch robot_launch simulation.launch.py world_file:=maze_test mode:=test
```

## Additional Notes
* Refer to the README files in each package for more details on their specific functionalities and usage.
# Tezeusz-Bot
