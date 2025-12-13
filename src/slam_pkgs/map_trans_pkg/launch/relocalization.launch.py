from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=[
                    {
                        "map_voxel_size": 0.3,  # 全局地图体素降采样尺寸（米）
                        "scan_voxel_size": 0.1,  # 当前扫描体素降采样尺寸（米）
                        "localization_freq": 1.0,  # 定位频率（Hz）
                        "localization_threshold": 0.8,  # ICP 配准成功阈值（fitness score）
                        "fov": 6.28318530718,  # 视野角度（弧度），默认 2π（360°）
                        "fov_far": 30.0,  # 视野最大距离（米）
                        "tf_pub_freq": 50.0,  # TF 广播频率（Hz）
                    }
                ],
            )
        ]
    )
