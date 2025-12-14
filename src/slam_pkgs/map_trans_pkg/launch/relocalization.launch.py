import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    r2n_bringup_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    pcd_file_path = os.path.join(r2n_bringup_pkg_share, "pcds", "test.pcd")

    # Get package directories
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
            # Relocalization node — now loads map internally
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=[
                    {
                        "global_map_path": pcd_file_path,   # 直接传给节点
                        "map_voxel_size": 0.3,
                        "scan_voxel_size": 0.1,
                        "localization_freq": 1.0,
                        "localization_threshold": 0.8,
                        "fov": 6.28318530718,
                        "fov_far": 30.0,
                        "tf_pub_freq": 50.0,
                    }
                ],
            ),
        ]
    )