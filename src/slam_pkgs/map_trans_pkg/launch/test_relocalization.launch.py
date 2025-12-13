# test_relozalization.launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取各包的共享目录
    virtual_lidar_pkg_share = get_package_share_directory('virtual_lidar_driver_pkg')
    map_trans_pkg_share = get_package_share_directory('map_trans_pkg')
    r2n_bringup_pkg_share = get_package_share_directory('r2n_bringup_pkg')

    # 定义PCD文件路径
    pcd_file_path = os.path.join(r2n_bringup_pkg_share, 'pcds', 'sim_map.pcd')
    
    # 包含virtual lidar driver的launch文件
    virtual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(virtual_lidar_pkg_share, 'launch', 'virtual_lidar.launch.py')
        ),
        # 指定参数 lidar_topic: "/cloud_registered"
        parameters=[{
            'lidar_topic': '/cloud_registered'
        }]
    )

    # 启动map_publisher_node节点
    map_publisher_node = Node(
        package='map_trans_pkg',
        executable='map_publisher_node',
        name='map_publisher',
        output='screen',
        parameters=[{
            'pcd_file': pcd_file_path
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 包含relocalization的launch文件
    relocalization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_trans_pkg_share, 'launch', 'relocalization.launch.py')
        )
    )

    return LaunchDescription([
        LogInfo(msg="Starting integrated navigation system"),
        
        # 启动各个组件
        virtual_lidar_launch,
        map_publisher_node,
        relocalization_launch,
        
        LogInfo(msg="All components started successfully")
    ])