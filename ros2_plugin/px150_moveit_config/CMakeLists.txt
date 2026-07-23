cmake_minimum_required(VERSION 3.8)
project(px150_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY config launch srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
