import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_driver_pkg_dir = get_package_share_directory("livox_ros_driver2")
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # ---------------------------------------------------------
    # [修正] 处理雷达倒装的 TF 变换
    # ---------------------------------------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link",
        # 参数: x y z yaw pitch roll frame_id child_frame_id
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "body", "base_link"],
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    launch_lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_pkg_dir, "launch", "msg_MID360_launch.py")
        )
    )
    
    # # ---------------------------------------------------------
    # # [新增] 方块识别节点
    # # ---------------------------------------------------------
    # # 注意 package 必须是你刚才修改 CMakeLists.txt 的那个包名 (map_trans_pkg)
    # cube_detector_node = Node(
    #     package='map_trans_pkg', 
    #     executable='cube_detector',  # 对应 CMakeLists.txt 中 RENAME 后的名字
    #     name='cube_detector',
    #     output='screen',
    #     # 注意：默认订阅 /livox/lidar_pc2。
    #     # 如果你想用 Fast_LIO 去畸变后的点云，取消下面这行的注释：
    #     remappings=[('/livox/lidar_pc2', '/cloud_registered')]
    # )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(launch_lidar_driver)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    
    # [关键步骤] 加入 TF 和 识别节点
    ld.add_action(static_tf_node)
    # ld.add_action(cube_detector_node)

    return ld