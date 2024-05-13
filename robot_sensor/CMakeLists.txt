cmake_minimum_required(VERSION 3.5)
project(robot_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########### Find dependencies #############

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

########### Create executable #############

add_executable(robot_sensor src/robot_sensor.cpp)

########### Link dependencies #############

ament_target_dependencies(
  robot_sensor 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  tf2_ros)

########### Install executable #############
install(TARGETS
  robot_sensor
  DESTINATION lib/${PROJECT_NAME}
)

########### Install directories #############

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

########### Ament package #############
ament_package()
