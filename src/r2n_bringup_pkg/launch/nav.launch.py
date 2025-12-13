import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    map_trans_pkg_share = get_package_share_directory("map_trans_pkg")

    reloc_launch_path = os.path.join(
        map_trans_pkg_share, "launch", "relocalization.launch.py"
    )
    nav2_params_file = os.path.join(nav_pkg_share, "params", "nav2_params.yaml")
    map_yaml_file = os.path.join(nav_pkg_share, "maps", "test.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(reloc_launch_path),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"),
                     "launch", "navigation_launch.py"]
                )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "autostart": "true",
                    "params_file": nav2_params_file,
                    "map": map_yaml_file,
                    "slam": "false",
                    "localization": "false",
                }.items(),
            ),
            Node(
                package="map_trans_pkg",
                executable="pointcloud2_to_laserscan",
                name="pointcloud2_to_laserscan",
                output="screen",
                parameters=[
                    {
                        "pc_topic": "/cloud_registered",
                        "laserscan_topic": "/scan",
                        "min_height": -0.2,
                        "max_height": 0.2,
                        "min_dist": 0.1,
                        "max_dist": 10.0,
                        "angle_resolution_deg": 1.0,
                        "angle_min_deg": -180.0,
                        "angle_max_deg": 180.0,
                        "filter_mean_k": 5,
                        "filter_stddev": 1.0,
                        "grid_resolution": 0.05,
                    }
                ],
            ),
            Node(
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
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(nav_pkg_share, "rviz", "nav2.rviz")],
                output="screen",
            ),
        ]
    )
