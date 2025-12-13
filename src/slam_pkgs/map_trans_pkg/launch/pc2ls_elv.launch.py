from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="map_trans_pkg",
                executable="pointcloud2laserscan_elv",
                name="pointcloud2laserscan_elv",
                output="screen",
                parameters=[
                    {
                        "pc_topic": "/cloud_registered",
                        "laserscan_topic": "/laserscan",
                        "pgm_topic": "",  # 留空表示不发布 PGM 地图
                        "elv_topic": "",  # 留空表示不发布高程图
                        "min_height": -0.2,
                        "max_height": 0.2,
                        "min_dist": 0.1,
                        "max_dist": 10.0,
                        "angle_resolution_deg": 1.0,
                        "angle_min_deg": -180.0,
                        "angle_max_deg": 180.0,
                        "passable_thresh": 0.1,  # 可通行高度差阈值（米）
                        "grid_resolution": 0.1,  # PGM 网格分辨率（米）
                        "elv_ratio": 2.0,  # 高程图分辨率是 PGM 的 elv_ratio 倍（即更精细）
                        "filter_kernel": 5,  # 形态学滤波核大小（奇数）
                        "top_k": 5,  # 注意：当前代码中 top_k 未被使用，但已声明
                    }
                ],
            )
        ]
    )
