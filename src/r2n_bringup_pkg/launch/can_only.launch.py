import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Serial CAN bridge node (connects ROS ↔ serial port)
            Node(
                package="can_serial_pkg",
                executable="can_serial_node",
                name="serial_can_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB0",
                        "baud_rate": 250*10000, 
                        "can_downward_topic": "/msg_to_can",
                        "can_upward_topic": "/msg_from_can",
                    }
                ],
            ),

            # 2. Middle coding node: /cmd_vel → encoded CAN frame → /msg_to_can
            Node(
                package="can_serial_pkg",
                executable="middle_coding_node",
                name="middle_coding_node",
                output="screen",
            ),

            # 3. rqt robot steering
            Node(
                package="rqt_robot_steering",
                executable="rqt_robot_steering",
                name="rqt_robot_steering",
                output="screen",
            ),
        ]
    )