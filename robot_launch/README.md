# robot_launch package

This package contains launch files for running various components of the robot system, including simulation, navigation, teleoperation, and more. It provides a convenient way to start and configure multiple nodes and processes involved in robot operation.

## Launch Files
* **`simulation.launch.py`:**
    * This is the main launch file for starting the robot simulation environment.
    * It includes launch files for:
        * Gazebo simulation with different world files.
        * Robot driver for controlling robot movement.
        * Sensor processing for lidar data.
        * Navigation node in either training or testing mode.
        * Teleoperation node for keyboard control.
    * Arguments:
        * `world_file`: Specifies the name of the world file to load in Gazebo: maze_train or maze_test.
        * `mode`: Sets the robot's navigation mode. Choose from:
            * train: Enables manual control of the robot. During manual control, data is collected for training the navigation model.
            * test: Allows the robot to be controlled by the model you trained in train mode.
            * None: Free ride.


## Example

* Launch the simulation with the 'maze_train.world' file and with no navigation:

```bash
ros2 launch robot_launch simulation.launch.py world_file:=maze_train mode:=None
```

