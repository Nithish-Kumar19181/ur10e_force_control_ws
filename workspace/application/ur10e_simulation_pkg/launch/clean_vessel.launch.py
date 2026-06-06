#!/usr/bin/env python3
"""Launch the multi-layer vessel-cleaning skill (ur10e_circle_follow).

Loads all parameters (geometry, force PI, hard-spot, checkpoint, home angles)
from config/circle_follow_params.yaml. The cartesian_compliance_controller must
already be active before this is launched; the skill switches to the
joint_trajectory_controller itself for the between-layer tool flips and homing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ur10e_simulation_pkg")
    default_params = os.path.join(pkg, "config", "circle_follow_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the circle_follow_params.yaml parameter file.",
    )
    # Sim stamps /joint_states with Gazebo time, so MoveIt's current-state
    # monitor needs the node clock on sim time too. Set false on the real robot.
    sim_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock. Set false on real hardware.",
    )

    clean_node = Node(
        package="ur10e_simulation_pkg",
        executable="ur10e_circle_follow",
        name="circle_follow_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([params_arg, sim_arg, clean_node])
