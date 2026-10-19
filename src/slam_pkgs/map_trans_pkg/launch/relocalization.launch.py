import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 定义路径
    r2n_bringup_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    fast_lio_pkg_dir = get_package_share_directory("fast_lio")
    livox_driver_pkg_dir = get_package_share_directory("livox_ros_driver2")

    # PCD 文件路径
    pcd_file_path = os.path.join(r2n_bringup_pkg_share, "pcds", "test.pcd")
    
    # RViz 配置文件的路径
    rviz_config_path = os.path.join(r2n_bringup_pkg_share, "rviz", "nav2.rviz")

    return LaunchDescription(
        [
            # ---------------------------------------------------------
            # 1. 启动 Fast-LIO 里程计
            # ---------------------------------------------------------
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_pkg_dir, "launch", "mapping.launch.py")
                )
            ),

            # ---------------------------------------------------------
            # 2. 启动 Livox 雷达驱动
            # ---------------------------------------------------------
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_driver_pkg_dir, "launch", "msg_MID360_launch.py")
                )
            ),

            # # ---------------------------------------------------------
            # # 3. [新增] 方块识别与聚类节点 (适配 350mm 方块)
            # # ---------------------------------------------------------
            # Node(
            #     package='map_trans_pkg',
            #     executable='dbscan_clustering.py',
            #     name='cube_cluster_detector',
            #     output='screen',
            #     parameters=[{
            #         'eps': 0.18,              # 减小半径以区分方块和紧贴的台阶
            #         'voxel_size': 0.05,       # 2cm 下采样，保留方块几何特征
            #         'z_min': -0.4,            # 过滤地面，保留台阶上的物体
            #         'z_max': 0.8,             # 限制高度
            #         'min_cluster_size': 30,   # 过滤噪点
            #         'radius_filter': 6.0,     # 只看周围 3 米，提高算力效率
            #     }],
            #     # 如果 Fast-LIO 输出了去畸变点云，映射到这里
            #     remappings=[('/livox/lidar', '/cloud_registered')]
            # ),

            # ---------------------------------------------------------
            # 4. 启动重定位/地图发布节点 (原逻辑保留)
            # ---------------------------------------------------------
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=[{
                    "global_map_path": pcd_file_path,
                    "map_voxel_size": 0.3,
                    "scan_voxel_size": 0.1,
                    "localization_freq": 1.0,
                    "localization_threshold": 0.8,
                    "fov": 6.28318530718,  # 2π
                    "fov_far": 30.0,
                    "tf_pub_freq": 50.0,
                }]
            ),

            # ---------------------------------------------------------
            # 5. 静态地图可视化发布
            # ---------------------------------------------------------
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                name="map_visualization_publisher",
                output="screen",
                parameters=[{
                    "file_name": pcd_file_path,
                    "interval": 5.0,
                    "frame_id": "map",
                    "latch": True
                }],
                remappings=[
                    ("cloud_pcd", "/map_pointcloud")
                ]
            ),

            # ---------------------------------------------------------
            # 6. RViz2 可视化
            # ---------------------------------------------------------
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),

            # ---------------------------------------------------------
            # 7. 静态 TF (body -> base_link)
            # ---------------------------------------------------------
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
            ),
        ]
    )