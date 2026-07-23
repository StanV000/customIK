cmake_minimum_required(VERSION 3.8)
project(custom_ik_moveit_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(custom_ik_planner_lib
  src/custom_ik_planner.cpp
)
target_include_directories(custom_ik_planner_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(custom_ik_planner_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  control_msgs
  std_msgs
)

add_executable(custom_ik_planner_node src/custom_ik_planner_node.cpp)
target_link_libraries(custom_ik_planner_node custom_ik_planner_lib)
ament_target_dependencies(custom_ik_planner_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  control_msgs
  std_msgs
)

install(TARGETS custom_ik_planner_node custom_ik_planner_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
