cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########### Find dependencies #############
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

########### Create executable #############

add_executable(robot_driver src/robot_driver.cpp)

########### Link dependencies #############

ament_target_dependencies(robot_driver rclcpp geometry_msgs)

########### Install executable #############
install(TARGETS
  robot_driver
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
