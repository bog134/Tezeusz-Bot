import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry

from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import math
import numpy as np


class TeleopController(Node):
    """Node for controlling the robot using keyboard input."""

    def __init__(self):
        super().__init__('teleop_controller')

        # Initialize ROS publishers and subscribers
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            '/laser_data',
            self.lidar_callback,
            10
        )

        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odometry_callback,
            10
        )

        self.movement_publisher = self.create_publisher(
            String,
            '/move_data',
            10
        )

        # Initialize keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            suppress=False
        )
        self.keyboard_listener.start()

        # Initialize state variables
        self.blocked_direction = None  # Direction to maintain until unblocked
        self.robot_pose = (0.0, 0.0)  # (x, y) position of the robot
        self.is_pose_updated = False  # Flag to indicate if pose has been updated
        self.distance_traveled = 0.0  # Distance traveled since last pose update 
        self.current_movement = 'stop'  # Current movement command 
        self.movement_mode = None  # Current movement mode (e.g., speed/distance)
        self.current_yaw = None #Current yaw

        self.get_logger().info('Publishing: "%s"' % "Robot teleop node initialized")

    def lidar_callback(self, lidar_data_msg):
        """Process lidar data and determine movement commands."""

        movement_command = String()

        # Check if path is clear and unblock if previously blocked
        if np.all(np.array(lidar_data_msg.data[90:270]) == np.inf) and self.blocked_direction is not None:
            movement_command.data = "stop"
            self.blocked_direction = None
        else:
            movement_command.data = "stop"

            # Check for significant difference in lidar readings (potential obstacle)
            if abs(lidar_data_msg.data[90] - lidar_data_msg.data[91]) > 0.5:
                self.is_pose_updated = True

            def turn(direction, target_angle):
                """Rotates the robot by 90 degrees (1.57 radians)."""
                movement_command.data = direction

                # Adjust yaw angle based on current and target orientation
                if direction == "left":
                    if self.robot_yaw >= 1.57 and self.current_yaw < 0:
                        self.current_yaw = 2 * math.pi + self.current_yaw
                    if self.current_yaw >= target_angle + self.robot_yaw:
                        self.blocked_direction = None
                elif direction == "right":
                    if self.robot_yaw <= -1.57 and self.current_yaw > 0:
                        self.current_yaw = -2 * math.pi + self.current_yaw
                    if self.current_yaw <= target_angle + self.robot_yaw:
                        self.blocked_direction = None

            def move_forward():
                """Helper function to handle forward movement based on mode."""

                #The robot drives straight until it's 0.35 away form a wall (it is capable of performing a 180-degree turn when facing a wall)
                if self.movement_mode == 1:
                    if lidar_data_msg.data[180] > 0.35:
                        movement_command.data = "forward"
                    else:
                        self.blocked_direction = None
                        movement_command.data = "stop"

                #The robot drives straight until it's 0.2 away form a wall (it is capable of performing a 90-degree turn when facing a wall)
                elif self.movement_mode == 2:
                    if lidar_data_msg.data[180] > 0.2:
                        movement_command.data = "forward"
                    else:
                        self.blocked_direction = None
                        movement_command.data = "stop"

                # The robot halts at a distance from the edge of the turn sufficient to allow for a subsequent 180-degree rotation away from the wall
                elif self.movement_mode == 3:
                    if self.distance_traveled >= 0.45:
                        movement_command.data = "stop"
                        self.blocked_direction = None
                        self.robot_pose = (0.0, 0.0)
                        self.distance_traveled = 0.0
                    else:
                        movement_command.data = "forward"

                # The robot halts at a distance from the edge of the turn sufficient to allow for a subsequent 90-degree rotation away from the wall
                elif self.movement_mode == 4:
                    if self.distance_traveled >= 0.30:
                        movement_command.data = "stop"
                        self.blocked_direction = None
                        self.robot_pose = (0.0, 0.0)
                        self.distance_traveled = 0.0
                    else:
                        movement_command.data = "forward"

            # Set initial movement command 
            movement_command.data = self.current_movement

            # Update robot yaw and handle blocked movements
            if self.blocked_direction is None:
                self.robot_yaw = self.current_yaw
            elif self.blocked_direction == "left":
                turn("left", 1.57)
            elif self.blocked_direction == "right":
                turn("right", -1.57)
            elif self.blocked_direction == "forward":
                move_forward()

            # Reset current movement and publish command
            self.current_movement = 'stop'
            self.movement_publisher.publish(movement_command)

    def odometry_callback(self, odometry_msg):
        """Process odometry data and update robot pose."""

        # Extract orientation quaternion and convert to yaw angle 
        orientation_quat = [
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y, 
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        ]
        rotation = R.from_quat(orientation_quat)
        self.current_yaw = round(rotation.as_euler('xyz', degrees=False)[-1], 2)

        # Calculate distance traveled since last pose update
        if self.robot_pose != (0.0, 0.0):
            self.distance_traveled = math.sqrt(
                (odometry_msg.pose.pose.position.x - self.robot_pose[0])**2 +
                (odometry_msg.pose.pose.position.y - self.robot_pose[1])**2
            )

        # Update robot pose if flag is set 
        if self.is_pose_updated:
            self.robot_pose = (
                odometry_msg.pose.pose.position.x, 
                odometry_msg.pose.pose.position.y
            )
            self.is_pose_updated = False

        # Log current yaw angle for debugging 
        self.get_logger().info('Publishing: "%s"' % f'Current yaw: {self.current_yaw}')

    def on_key_press(self, key):
        """Handle keyboard input and update movement commands."""

        # Get key character or name
        try:
            key_char = key.char
        except AttributeError:
            key_char = key.name

        # Update movement command based on key press
        if key_char == 'w':  # Forward
            self.current_movement = "forward"
            self.blocked_direction = None
        elif key_char == 's':  # Backward 
            self.current_movement = "backward"
            self.blocked_direction = None
        elif key_char == 'a':  # Left 
            self.current_movement = "left"
            self.blocked_direction = None
        elif key_char == 'd':  # Right
            self.current_movement = "right"
            self.blocked_direction = None

        # Set blocked direction for turning
        elif key_char == 'left':
            self.blocked_direction = "left"
        elif key_char == 'right':
            self.blocked_direction = "right"

        # Set forward movement mode
        elif key_char == '1': 
            self.blocked_direction = "forward"
            self.movement_mode = 1
        elif key_char == '2': 
            self.blocked_direction = "forward"
            self.movement_mode = 2
        elif key_char == '3': 
            self.blocked_direction = "forward"
            self.movement_mode = 3
        elif key_char == '4': 
            self.blocked_direction = "forward"
            self.movement_mode = 4

def main(args=None):
    rclpy.init(args=args)
    teleop_controller = TeleopController()
    rclpy.spin(teleop_controller)
    teleop_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()