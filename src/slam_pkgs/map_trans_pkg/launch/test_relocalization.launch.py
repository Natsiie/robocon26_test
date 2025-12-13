# test_relocalization.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 获取各包的共享目录
    virtual_lidar_pkg_share = get_package_share_directory("virtual_lidar_driver_pkg")
    map_trans_pkg_share = get_package_share_directory("map_trans_pkg")
    r2n_bringup_pkg_share = get_package_share_directory("r2n_bringup_pkg")

    # 定义 PCD 文件路径（用于全局地图发布）
    pcd_file_path = os.path.join(r2n_bringup_pkg_share, "pcds", "sim_map.pcd")

    # 包含 virtual_lidar_driver 的 launch 文件，并正确传递参数覆盖
    virtual_lidar_include = Node(
        package="virtual_lidar_driver_pkg",
        executable="virtual_lidar_driver_node",  # 直接启动节点并传递参数（推荐方式）
        name="virtual_lidar_driver_node",
        output="screen",
        parameters=[
            {
                "lidar_topic": "/cloud_registered",  # 覆盖 config 中的 /livox/lidar_pc2
                "pcd_path": "$(find r2n_bringup_pkg)/pcds/sim_map.pcd",  # 确保一致
                # 如需其他覆盖参数，可继续添加
            }
        ],
        remappings=[
            # 可选：如果需要 remap 其他话题
        ],
    )

    # 额外包含 pc2_to_livoxcloud_node 和静态 TF（原 launch 中有）
    pc2_to_livoxcloud_node = Node(
        package="virtual_lidar_driver_pkg",
        executable="pc2_to_livoxcloud_node",
        name="pc2_to_livoxcloud",
        output="screen",
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_init"],
    )

    # 启动 PCD 发布节点（发布全局地图到 /map3d 以匹配 relocalization_node 的硬编码）
    pcd_publisher_node = Node(
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
    )
    relocalization_include = Node(
        package="map_trans_pkg",
        executable="relocalization_node",
        name="relocalization_node",
        output="screen",
        parameters=[  # 可选：覆盖默认参数
            {
                "map_voxel_size": 0.3,
                "scan_voxel_size": 0.1,
                "localization_freq": 1.0,
                "localization_threshold": 0.8,
                "fov": 6.28318530718,
                "fov_far": 30.0,
                "tf_pub_freq": 50.0,
            }
        ],
    )
    
    # rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(map_trans_pkg_share, "config", "relocalization.rviz")],
        output="screen",
    )

    return LaunchDescription(
        [
            LogInfo(msg="Starting test_relocalization launch..."),
            # 虚拟雷达驱动相关
            virtual_lidar_include,
            pc2_to_livoxcloud_node,
            camera_tf,
            # 全局地图发布
            pcd_publisher_node,
            # 重定位节点
            relocalization_include,
            rviz,
            LogInfo(msg="All components for relocalization test started successfully!"),
        ]
    )
