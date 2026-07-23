#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"

namespace custom_ik_moveit_planner {

class CustomIKPlanner {
public:
  explicit CustomIKPlanner(const rclcpp::Node::SharedPtr &node);

  void planToPose(const geometry_msgs::msg::PoseStamped &target_pose);
  void setPlanningGroup(const std::string &planning_group);

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_ = "arm_group";
};

}  // namespace custom_ik_moveit_planner
