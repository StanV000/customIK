from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_description = LaunchConfiguration("robot_description")
    robot_description_semantic = LaunchConfiguration("robot_description_semantic")

    return LaunchDescription([
        DeclareLaunchArgument("robot_description", default_value="$(find-pkg-share px150_moveit_config)/urdf/px150.urdf.xacro"),
        DeclareLaunchArgument("robot_description_semantic", default_value="$(find-pkg-share px150_moveit_config)/srdf/px150.srdf"),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": robot_description_semantic},
                {"planning_plugin": "ompl_interface/OMPLPlanner"},
                {"request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"},
            ],
        ),
    ])
