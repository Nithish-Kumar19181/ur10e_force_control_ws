#!/usr/bin/env python3
"""Bring up the surface-cleaning path pipeline (cloud_processor + path_generator).

Run this alongside vertical_mixer_ur10e.launch.py (which provides the arm, the
D435i camera and TF). Then trigger a one-shot capture:

    ros2 service call /ur10e_vision/trigger std_srvs/srv/Trigger {}
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("ur10e_vision")
    default_params = os.path.join(pkg_share, "config", "vision_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur10e_vision"), "rviz", "cleaning_path.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="YAML with cloud_processor + path_generator parameters."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Launch a standalone RViz with the cleaning-path view."),

        Node(
            package="ur10e_vision",
            executable="cloud_processor",
            name="cloud_processor",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
        ),
        Node(
            package="ur10e_vision",
            executable="waypoint_generator_node",
            name="waypoint_generator",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
