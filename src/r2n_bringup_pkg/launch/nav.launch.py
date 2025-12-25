import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    nav_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    map_trans_pkg_share = get_package_share_directory("map_trans_pkg")
    fast_lio_pkg_dir = get_package_share_directory("fast_lio")
    livox_driver_pkg_dir = get_package_share_directory("livox_ros_driver2")

    # Map file paths (centralized here)
    pcd_file_path = os.path.join(nav_pkg_share, "pcds", "test.pcd")      # PCD for relocalization
    map_yaml_file = os.path.join(nav_pkg_share, "maps", "test.yaml")     # YAML + PGM for Nav2
    # nav2_params_file = os.path.join(nav_pkg_share, "params", "nav2_params.yaml")
    nav2_params_file = os.path.join(nav_pkg_share, "params", "nav2_params_without_rotation.yaml")
    rviz_config_path = os.path.join(nav_pkg_share, "rviz", "nav2.rviz")

    return LaunchDescription(
        [
            # === Livox Mid-360 Driver ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_driver_pkg_dir, "launch", "msg_MID360_launch.py")
                )
            ),

            # === Fast-LIO Mapping ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_pkg_dir, "launch", "mapping.launch.py")
                )
            ),

            # === Relocalization Node (directly launched, with PCD path) ===
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=[
                    {
                        "global_map_path": pcd_file_path,   # PCD map for relocalization
                        "map_voxel_size": 0.3,
                        "scan_voxel_size": 0.1,
                        "localization_freq": 1.0,
                        "localization_threshold": 0.8,
                        "fov": 6.28318530718,  # 2π
                        "fov_far": 30.0,
                        "tf_pub_freq": 50.0,
                    }
                ],
            ),

            # === Nav2 Bringup (localization only, no SLAM) ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
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
                    "use_lifecycle_manager": "false",
                }.items(),
            ),

            # === PointCloud to LaserScan (for Nav2 costmap) ===
            Node(
                package="map_trans_pkg",
                executable="pointcloud2_to_laserscan",
                name="pointcloud2_to_laserscan",
                output="screen",
                parameters=[
                    {
                        "pc_topic": "/cloud_registered_body",
                        "laserscan_topic": "/scan",
                        "output_frame": "body_2d",
                        "min_height": 0.2,
                        "max_height": 1.0,
                        "min_dist": 0.4,
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

            # === Static Map Publisher for Nav2 ===
            Node(
                package="map_trans_pkg",
                executable="map_publisher_node",
                name="map_for_nav2",
                output="screen",
                parameters=[
                    {
                        "map_path": map_yaml_file,      # Same YAML used by Nav2
                        "publish_topic": "/map",
                        "frame_id": "map",
                        "publish_interval": 10.0,
                    }
                ],
            ),
            # 1. Serial CAN bridge node (connects ROS ↔ serial port)
            Node(
                package="can_serial_pkg",          # Replace with your actual package name
                executable="can_serial_node",
                name="serial_can_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB0",   # Adjust if needed
                        "baud_rate": 2000000,
                        "can_downward_topic": "/msg_to_can",
                        "can_upward_topic": "/msg_from_can",
                    }
                ],
            ),

            # 2. Middle coding node: /cmd_vel → encoded CAN frame → /msg_to_can
            Node(
                package="can_serial_pkg",          # Same package as above
                executable="middle_coding_node",
                name="middle_coding_node",
                # output="screen",
                # No parameters needed — topics are hardcoded in the node
            ),
            # === RViz ===
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )