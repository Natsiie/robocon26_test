#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions.set_parameter import SetParameter


def generate_launch_description():

    r2n_bringup_pkg_dir = get_package_share_directory("r2n_bringup_pkg")
    virtual_robot_sim_pkg_dir = get_package_share_directory("virtual_robot_sim_pkg")

    declare_args = [
        DeclareLaunchArgument(
            "map_yaml_file",
            default_value=PathJoinSubstitution(
                [r2n_bringup_pkg_dir, "maps", "sim_map.yaml"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(r2n_bringup_pkg_dir, "rviz", "sim_nav2.rviz"),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                r2n_bringup_pkg_dir, "params", "nav2_params_sim.yaml"
            ),
        ),
        DeclareLaunchArgument("world_to_map_x", default_value="0.0"),
        DeclareLaunchArgument("world_to_map_y", default_value="10.0"),
        DeclareLaunchArgument("world_to_map_yaw", default_value="0.0"),
    ]
    SetParameter(name="use_sim_time", value=False)
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    params_file = LaunchConfiguration("params_file")
    wx = LaunchConfiguration("world_to_map_x")
    wy = LaunchConfiguration("world_to_map_y")
    wyaw = LaunchConfiguration("world_to_map_yaw")

    simulation_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [virtual_robot_sim_pkg_dir, "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "use_rviz": "false",
            "map_yaml_file": map_yaml_file,
            "map": LaunchConfiguration("map_yaml_file"),
        }.items(),
    )

    pc2ls = Node(
        package="map_trans_pkg",
        executable="pointcloud2_to_laserscan",
        name="pc2_to_scan",
        output="screen",
        parameters=[
            {
                "pc_topic": "/livox/lidar_pc2",
                "laserscan_topic": "/scan",
                "lidar_frame": "robot_lidar",
                "base_frame": "base_link",
                "output_frame": "body",
                "min_height": 0.1,
                "max_height": 1.0,
            }
        ],
    )
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        arguments=[wx, wy, "0", "0", "0", wyaw, "world", "map"],
        parameters=[{"use_sim_time": False}],
    )

    map_for_nav2 = Node(
        package="map_trans_pkg",
        executable="map_publisher_node",
        name="map_for_nav2",
        parameters=[
            {
                "map_path": map_yaml_file,
                "publish_topic": "/map",
                "frame_id": "map",
                "publish_interval": 10.0,
            }
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": params_file,
            "use_simulator": "false",
            "headless": "false",
            "map": map_yaml_file,
            "use_lifecycle_manager": "false",
            "slam": "false",
            "localization": "true",
        }.items(),
    )

    odom_node = Node(
        package="r2n_bringup_pkg",
        executable="sim_odom_publisher",
        name="sim_odom_publisher",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription(
        [
            *declare_args,
            simulation_full,
            pc2ls,
            world_to_map,
            map_for_nav2,
            nav2_bringup,
            odom_node,
            rviz,
        ]
    )
