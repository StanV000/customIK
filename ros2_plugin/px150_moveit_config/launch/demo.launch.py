from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_description = LaunchConfiguration("robot_description")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_description",
            default_value="$(find-pkg-share px150_moveit_config)/urdf/px150.urdf.xacro",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{ "robot_description": robot_description }],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": "$(find-pkg-share px150_moveit_config)/srdf/px150.srdf"},
                {"moveit_config": "$(find-pkg-share px150_moveit_config)"},
            ],
        ),
    ])
