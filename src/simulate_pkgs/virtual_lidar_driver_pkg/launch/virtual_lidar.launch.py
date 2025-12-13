from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("virtual_lidar_driver_pkg")
    params_file = os.path.join(pkg_share, "config", "virtual_lidar_config.yaml")

    node = Node(
        package="virtual_lidar_driver_pkg",
        executable="virtual_lidar_driver_node",
        name="virtual_lidar_driver_node",
        output="screen",
        parameters=[params_file],
    )

    # 添加pc2_to_livoxcloud节点
    pc2_to_livoxcloud_node = Node(
        package="virtual_lidar_driver_pkg",
        executable="pc2_to_livoxcloud_node",
        name="pc2_to_livoxcloud",
        output="screen",
    )
    
    # 发布从 world 到 camera_init 的 tf 
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_init"],
    )

    return LaunchDescription([node, pc2_to_livoxcloud_node,camera_tf])
