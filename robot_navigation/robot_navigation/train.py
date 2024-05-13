import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from robot_navigation.model.utils import save_data, clean_file

class DataCollector(Node):
    """Collects LiDAR data and movement commands for training the navigation model."""

    def __init__(self):
        super().__init__('data_collector')

        # Subscribers for movement commands and LiDAR data
        self.move_command_subscriber = self.create_subscription(
            String, 
            '/move_data',
            self.move_command_callback,
            10)

        self.lidar_data_subscriber = self.create_subscription(
            Float32MultiArray,
            '/laser_data',
            self.lidar_data_callback,
            10)

        self.lidar_data = None
        self.move_command = None
        clean_file()

    def move_command_callback(self, msg):
        """Handles incoming movement command messages."""
        self.move_command = msg.data
        self.collect_data()

    def lidar_data_callback(self, msg):
        """Handles incoming LiDAR data messages."""
        self.lidar_data = msg.data
        self.collect_data()

    def collect_data(self):
        """Saves data to file if both LiDAR and movement command are available and a robot is moving."""
        if self.lidar_data is not None and self.move_command is not None and self.move_command != 'stop':
            #self.get_logger().info(f"Saving data: LiDAR - {self.lidar_data[90]}, Command - {self.move_command}")
            save_data(self.lidar_data, self.move_command)
            self.lidar_data = None
            self.move_command = None

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)  
    data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()