import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    r2n_bringup_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    pcd_file_path = os.path.join(r2n_bringup_pkg_share, "pcds", "test.pcd")

    # Get package directories for the additional launch files
    fast_lio_pkg_dir = get_package_share_directory("fast_lio")
    livox_driver_pkg_dir = get_package_share_directory("livox_ros_driver2")

    return LaunchDescription(
        [
            # Include Fast-LIO mapping launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_pkg_dir, "launch", "mapping.launch.py")
                )
            ),
            # Include Livox ROS Driver2 Mid360 launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_driver_pkg_dir, "launch", "msg_MID360_launch.py")
                )
            ),
            # Relocalization node
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
            ),
            # PCD publisher node
            Node(
                package="map_trans_pkg",
                executable="pcd_publisher_node",  # python 节点编译后通常为此名（或 pcd_publisher_node）
                name="pcd_publisher",
                output="screen",
                parameters=[
                    {
                        "pcd_path": pcd_file_path,
                        "publish_topic": "/map3d",  # 关键：改为 relocalization_node 订阅的 /map3d
                        "frame_id": "map",
                        "publish_interval": 20.0,
                    }
                ],
            ),
        ]
    )
