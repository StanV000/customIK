#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_ik_moveit_planner/custom_ik_planner.hpp"

class CustomIKPlannerNode : public rclcpp::Node {
public:
  CustomIKPlannerNode()
  : Node("custom_ik_planner_node")
  {
    planner_ = std::make_shared<custom_ik_moveit_planner::CustomIKPlanner>(shared_from_this());

    RCLCPP_INFO(this->get_logger(), "Custom IK planner node initialized.");
    RCLCPP_INFO(this->get_logger(), "This package is the ROS 2 integration boundary for the C++ IK math in this repo.");
  }

private:
  std::shared_ptr<custom_ik_moveit_planner::CustomIKPlanner> planner_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomIKPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
