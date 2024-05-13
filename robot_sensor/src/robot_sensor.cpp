#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class LaserScan : public rclcpp::Node {
 public:
  LaserScan() : Node("laser_scan") {
    // Create a subscriber to the laser scan topic with SensorData QoS
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/demo/laser/out", rclcpp::SensorDataQoS(),
        std::bind(&LaserScan::scan_callback, this, std::placeholders::_1));

    // Create a publisher for processed laser data
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/laser_data", 10);

    RCLCPP_INFO(this->get_logger(), "Robot sensor node initialized");
  }

 private:
  // Callback function for processing laser scan data
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Create a Float32MultiArray message
    auto msg1 = std_msgs::msg::Float32MultiArray();

    // Copy laser scan ranges to the message data
    msg1.data = msg->ranges;

    // Publish the processed data
    publisher_->publish(msg1);

    // (Optional) Print information about the data for debugging
    // std::stringstream ss;
    // ss << msg->ranges[180];
    // std::string msg_str = "Distance: " + ss.str() + "\n";
    // RCLCPP_INFO(this->get_logger(), msg_str.c_str());
  }

  // Subscriber for laser scan data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  // Publisher for processed laser data
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScan>());
  rclcpp::shutdown();
  return 0;
}