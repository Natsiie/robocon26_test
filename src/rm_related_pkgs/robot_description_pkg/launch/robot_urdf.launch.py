# robot_description_pkg/launch/robot_urdf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 获取包目录路径
    pkg_share = get_package_share_directory("robot_description_pkg")

    # 定义URDF文件路径
    default_urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")

    # 声明启动参数
    urdf_file = DeclareLaunchArgument(
        "urdf_file",
        default_value=default_urdf_path,
        description="Path to robot URDF file",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # 读取URDF文件内容
    with open(default_urdf_path, "r") as infp:
        robot_desc = infp.read()

    # 创建节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_frequency": 100.0,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rate": 100.0,  # 添加发布频率参数
            }
        ],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "display_robot.rviz")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            urdf_file,
            use_sim_time,
            robot_state_publisher_node,
            joint_state_publisher_node,
            # rviz2_node,
        ]
    )
