from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="map_trans_pkg",
                executable="pointcloud2_to_laserscan",
                name="pointcloud2_to_laserscan",
                output="screen",
                parameters=[
                    {
                        "pc_topic": "/cloud_registered",
                        "laserscan_topic": "/laserscan",
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
            )
        ]
    )
