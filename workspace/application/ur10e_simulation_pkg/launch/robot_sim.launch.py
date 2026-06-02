import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ur10e_pkg_share = get_package_share_directory("ur10e_simulation_pkg")

    bowl_sdf_template = os.path.join(ur10e_pkg_share, "model", "bowl.sdf")
    bowl_sdf_spawn = "/tmp/bowl_spawn.sdf"

    with open(bowl_sdf_template, "r", encoding="utf-8") as f:
        bowl_sdf_content = f.read().replace("PACKAGE_PATH", ur10e_pkg_share)
    with open(bowl_sdf_spawn, "w", encoding="utf-8") as f:
        f.write(bowl_sdf_content)

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ur10e_testing_pkg"),
        "config",
        "controller_manager.yaml"
    ])

    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                "launch",
                "ur_sim_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "controllers_file": controllers_yaml,
            "moveit_config_package": "ur10e_simulation_pkg",
            "moveit_config_file": "ur10e.srdf.xacro",
        }.items()
    )

    spawn_cartesian_motion = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_motion_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    spawn_cartesian_force = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_force_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    spawn_cartesian_compliance = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_compliance_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    delayed_cartesian_start = TimerAction(
        period=2.0,
        actions=[
            spawn_cartesian_motion,
            spawn_cartesian_force,
            spawn_cartesian_compliance,
        ]
    )

    spawn_bowl = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "bowl",
                    "-file", bowl_sdf_spawn,
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                    "-Y", "0.0"
                ],
                output="screen",
            )
        ]
    )

    # for ign gazebo
    # spawn_bowl = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package="ros_gz_sim",
    #             executable="create",
    #             arguments=[
    #                 "-entity", "bowl",
    #                 "-file", bowl_sdf_spawn,
    #                 "-x", "0.1",
    #                 "-y", "0.003",
    #                 "-z", "0.0",
    #                 "-Y", "0.0"
    #             ],
    #             output="screen",
    #         )
    #     ]
    # )

    filtered_force_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_force_filter",
        name="ur10e_force_filter",
        output="screen"
    )

    # for ign gazebo
    # filtered_force_gz_node = Node(
    #     package="ur10e_testing_pkg",
    #     executable="ur10e_force_filter_gz",
    #     name="ur10e_force_filter_gz",
    #     output="screen"
    # )

    marker_array_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_marker_array",
        name="ur10e_marker_array",
        output="screen"
    )

    delayed_marker_node = TimerAction(
        period=2.0,
        actions=[marker_array_node]
    )

    # for ign gazebo bridge for force torque sensor
    # force_torque_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/wrench@geometry_msgs/msg/Wrench@gz.msgs.Wrench"
    #     ],
    #     output="screen"
    # )

    return LaunchDescription([
        ur_sim_moveit,
        delayed_cartesian_start,
        delayed_marker_node,
        spawn_bowl,
        filtered_force_node,        # for classic gazebo
        # filtered_force_gz_node,   # for ign gazebo
        # force_torque_bridge,      # for ign gazebo
    ])
