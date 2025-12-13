#!/usr/bin/env python3
# virtual_robot_sim_pkg/launch/simulation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("r2n_bringup_pkg"), "maps", "sim_map.yaml"]
        ),
        description="Path to the map YAML file (used for both world_map and nav map)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz"
    )

    # 获取参数
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # === RViz ===
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("virtual_robot_sim_pkg"),
                "rviz",
                "display.rviz",
            ),
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_rviz_arg,
            map_yaml_file_arg,
            # 全局 use_sim_time
            SetParameter(name="use_sim_time", value=use_sim_time),
            # 1. 机器人 URDF
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("robot_description_pkg"),
                            "launch",
                            "robot_urdf.launch.py",
                        ]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            # 2. 虚拟机器人运动仿真（发布 world -> base_link）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("virtual_robot_sim_pkg"),
                            "launch",
                            "virtual_robot_sim.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world_tf": "world",
                    "robot_tf": "base_link",
                }.items(),
            ),
            # 3. 虚拟激光雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("virtual_lidar_driver_pkg"),
                            "launch",
                            "virtual_lidar.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world_frame": "world",
                    "robot_frame": "base_link",
                }.items(),
            ),
            # 发布 /world_map，frame_id = world，周期发送
            Node(
                package="map_trans_pkg",
                executable="map_publisher_node",
                name="world_map_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_yaml_file,
                        "publish_topic": "/world_map",
                        "frame_id": "world",
                        "publish_interval": 5.0,
                    }
                ],
            ),
            # 6. RViz
            rviz_node,
            # 7. 遥控手柄
            # Node(
            #     package="rqt_robot_steering",
            #     executable="rqt_robot_steering",
            #     name="rqt_robot_steering",
            #     output="screen",
            # ),
        ]
    )
