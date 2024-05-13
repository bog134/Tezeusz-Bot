#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

class EntitySpawner : public rclcpp::Node {
 public:
  // Constructor initializes the node with the name "entity_spawner"
  EntitySpawner() : Node("entity_spawner") {
    // Create a client for the `/spawn_entity` service
    spawn_entity_client_ =
        this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Wait for the service to become available
    if (!spawn_entity_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
      throw std::runtime_error("Service not available");
    }
    RCLCPP_INFO(this->get_logger(), "Robot gazebo node initialized");
  }

  // Spawns a robot in Gazebo with the specified parameters
  bool SpawnEntity(const std::string& name, const std::string& robot_namespace,
                   const std::string& model_path, float x, float y, float z,
                   float o_w, float o_x, float o_y, float o_z) {
    // Prepare the request message
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = name;
    request->robot_namespace = robot_namespace;

    // Load the model from the specified SDF file
    std::ifstream ifs(model_path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    request->xml = buffer.str();

    // Set the initial pose of the robot
    request->initial_pose.position.x = x;
    request->initial_pose.position.y = y;
    request->initial_pose.position.z = z;
    request->initial_pose.orientation.w = o_w;
    request->initial_pose.orientation.x = o_x;
    request->initial_pose.orientation.y = o_y;
    request->initial_pose.orientation.z = o_z;

    // Send the request asynchronously and wait for the response
    auto future = spawn_entity_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);

    // Check the result of the spawn request
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(),
                  "Spawn response: %s",
                  future.get()->status_message.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Spawn failed");
      return false;
    }
  }

 private:
  // Client for the `/spawn_entity` service
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_entity_client_;
};

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Get the path to the robot model SDF file
  std::string model_path =
      ament_index_cpp::get_package_share_directory("robot_description") +
      "/models/pioneer2dx/model.sdf";

  // Create an instance of the EntitySpawner node
  auto entity_spawner = std::make_shared<EntitySpawner>();

  // Check if the correct number of arguments are provided
  if (argc >= 7) {
    // Spawn the entity with the provided arguments
    entity_spawner->SpawnEntity(argv[1],
                                argv[2],
                                model_path,
                                std::stof(argv[3]),
                                std::stof(argv[4]),
                                std::stof(argv[5]),
                                std::stof(argv[6]),
                                std::stof(argv[7]),
                                std::stof(argv[8]),
                                std::stof(argv[9]));

    // Spin the node to process callbacks
    rclcpp::spin(entity_spawner);
  } else {
    RCLCPP_ERROR(entity_spawner->get_logger(), "Invalid number of arguments");
  }

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}