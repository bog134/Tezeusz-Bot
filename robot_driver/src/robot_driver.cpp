#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Define the node class as "RobotDriver"
class RobotDriver : public rclcpp::Node {
 public:
  // Constructor initializes the node with the name "robot_driver"
  RobotDriver()
      : Node("robot_driver") {
    // Create a publisher for velocity commands on the topic "/demo/cmd_vel" with a queue size of 10
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 10);

    // Create a subscription to the topic "/move_data" with a queue size of 10
    // Bind the 'command_callback' function to handle incoming messages
    command_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/move_data", 10, std::bind(&RobotDriver::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Robot driver node initialized");
  }

 private:
  // Callback function to process incoming command messages
  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Create a Twist message to store velocity commands
    auto twist_msg = geometry_msgs::msg::Twist();

    // Set a default linear speed
    float linear_speed = 0.1;

    // Parse the incoming command message and set appropriate velocities
    if (msg->data == "forward") {
      twist_msg.linear.x = linear_speed;
      twist_msg.angular.z = 0.0;
    } else if (msg->data == "backward") {
      twist_msg.linear.x = -linear_speed /10;
      twist_msg.angular.z = 0.0;
    } else if (msg->data == "left") {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = linear_speed;
    } else if (msg->data == "right") {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = -linear_speed;
    } else {
      // Stop the robot for any unknown command
      // RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", msg->data.c_str());
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    // Publish the velocity command
    velocity_publisher_->publish(twist_msg);

    // // Log the published velocity command
    // RCLCPP_INFO(this->get_logger(), "Publishing: linear_x: %.2f, angular_z: %.2f", 
    //             twist_msg.linear.x, twist_msg.angular.z);
  }

  // Declare publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
};

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create a shared pointer to the RobotDriver node
  auto robot_driver_node = std::make_shared<RobotDriver>();

  // Spin the node to process callbacks and messages
  rclcpp::spin(robot_driver_node);

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}