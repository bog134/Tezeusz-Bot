import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from robot_navigation.model.utils import predict, initiate_model

class NavigationController(Node):
    """Predicts movement commands based on LiDAR data and publishes them to the robot."""

    def __init__(self, trained_model = False):
        super().__init__('navigation_controller')

        # Subscribers for LiDAR data
        self.lidar_data_subscriber = self.create_subscription(
            Float32MultiArray,
            '/laser_data',
            self.lidar_data_callback,
            10)

        # Publisher for predicted movement commands
        self.move_command_publisher = self.create_publisher(
            String, 
            '/move_data', 
            10)

        self.last_move_command = None

        #Initializes the AlexNet navigation model.
        initiate_model()

    def lidar_data_callback(self, msg):
        """Predicts movement commands based on LiDAR data and publishes them."""
        predicted_command = predict(msg.data)

        # Avoid immediate turns in opposite directions - issue with maze corners
        if (self.last_move_command == 'left' and predicted_command == 'right') or (self.last_move_command == 'right' and predicted_command == 'left'):
            predicted_command = 'left'  

        self.last_move_command = predicted_command
        self.get_logger().info(f"Publishing movement command: {predicted_command}")

        move_command_msg = String()
        move_command_msg.data = predicted_command
        self.move_command_publisher.publish(move_command_msg) 

def main(args=None):
    rclpy.init(args=args)
    navigation_controller = NavigationController()
    rclpy.spin(navigation_controller)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()