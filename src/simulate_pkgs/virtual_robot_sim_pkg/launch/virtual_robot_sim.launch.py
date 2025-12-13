#!/usr/bin/env python3
# virtual_robot_sim_pkg/launch/virtual_robot_sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # 声明启动参数
    map_topic_arg = DeclareLaunchArgument(
        "map_topic",
        default_value=TextSubstitution(text="/world_map"),
        description="Topic for map data",
    )

    cmd_topic_arg = DeclareLaunchArgument(
        "cmd_topic",
        default_value=TextSubstitution(text="/cmd_vel"),
        description="Topic for velocity commands",
    )

    world_tf_arg = DeclareLaunchArgument(
        "world_tf",
        default_value=TextSubstitution(text="world"),
        description="World frame TF name",
    )

    robot_tf_arg = DeclareLaunchArgument(
        "robot_tf",
        default_value=TextSubstitution(text="base_link"),
        description="Robot frame TF name",
    )

    init_pose_arg = DeclareLaunchArgument(
        "init_pose",
        default_value=TextSubstitution(text="[0.0, 0.0, 0.0, 0.0, 0.0,0.0]"),
        description="Initial pose as [x,y,z,roll,pitch,yaw]",
    )

    update_rate_arg = DeclareLaunchArgument(
        "update_rate",
        default_value=TextSubstitution(text="100.0"),
        description="Update rate in Hz",
    )

    # 创建节点
    virtual_robot_node = Node(
        package="virtual_robot_sim_pkg",
        executable="virtual_robot_sim_node",
        name="virtual_robot_sim_node",
        output="screen",
        parameters=[
            {
                "map_topic": LaunchConfiguration("map_topic"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "world_tf": LaunchConfiguration("world_tf"),
                "robot_tf": LaunchConfiguration("robot_tf"),
                "init_pose": LaunchConfiguration("init_pose"),
                "update_rate": LaunchConfiguration("update_rate"),
            }
        ],
    )

    return LaunchDescription(
        [
            map_topic_arg,
            cmd_topic_arg,
            world_tf_arg,
            robot_tf_arg,
            init_pose_arg,
            update_rate_arg,
            virtual_robot_node,
        ]
    )
