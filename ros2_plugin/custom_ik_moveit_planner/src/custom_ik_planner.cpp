#include "custom_ik_moveit_planner/custom_ik_planner.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"

namespace custom_ik_moveit_planner {

CustomIKPlanner::CustomIKPlanner(const rclcpp::Node::SharedPtr &node)
: node_(node)
{
}

void CustomIKPlanner::setPlanningGroup(const std::string &planning_group)
{
  planning_group_ = planning_group;
}

void CustomIKPlanner::planToPose(const geometry_msgs::msg::PoseStamped &target_pose)
{
  (void)target_pose;
  RCLCPP_INFO(node_->get_logger(), "Received target pose for planning group '%s'.", planning_group_.c_str());
  RCLCPP_INFO(node_->get_logger(), "This stub is where the custom IK math can be connected to MoveIt planning.");
}

}  // namespace custom_ik_moveit_planner
