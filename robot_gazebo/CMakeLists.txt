cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########### Find dependencies #############
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

########### Create executable #############
add_executable(robot_gazebo src/robot_gazebo.cpp)

########### Link dependencies #############

ament_target_dependencies(robot_gazebo 
  rclcpp 
  gazebo_msgs 
  std_msgs
  )

########### Install executable #############
install(TARGETS
  robot_gazebo
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

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

########### Ament package #############
ament_package()
