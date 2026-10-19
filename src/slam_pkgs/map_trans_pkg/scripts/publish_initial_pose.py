#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import time
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw / 2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('publish_initial_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        
        # 延时 6 秒，等待定位节点加载完地图
        time.sleep(6.0)
        self.publish_zero_pose()

    def publish_zero_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # 告诉算法：我认为我在地图坐标系的...

        # 这里假设启动位置是 (0, 0, 0)，朝向是 0
        q = quaternion_from_euler(0, 0, 0)
        msg.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        
        # 连发 3 次，防止丢包
        for i in range(3):
            self.publisher_.publish(msg)
            self.get_logger().info(f'Auto-publishing Initial Pose (0,0,0)... [{i+1}/3]')
            time.sleep(0.5)
        
        # 发完就自杀，任务完成
        self.get_logger().info('Done. Shutting down self.')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    try:
        node = InitialPosePublisher()
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()