#!/usr/bin/env python3
import os
import re
import open3d as o3d
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_share_directory


class SavePCDNode(Node):
    def __init__(self):
        super().__init__("save_pcd_node")

        # 声明参数并设置默认值
        self.declare_parameter("pc_topic", "/Laser_map")
        self.declare_parameter("pcd_path", "./scan.pcd")

        # 获取参数值
        pc_topic = self.get_parameter("pc_topic").get_parameter_value().string_value
        pcd_path = self.get_parameter("pcd_path").get_parameter_value().string_value

        # 解析路径中的 $(find package) 语法
        pcd_path = self.resolve_package_path(pcd_path)

        self.get_logger().info(f"订阅话题: {pc_topic}")
        self.get_logger().info(f"保存路径: {pcd_path}")

        # 确保目录存在
        directory = os.path.dirname(pcd_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
            self.get_logger().info(f"创建目录: {directory}")

        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2, pc_topic, self.callback, 10
        )

        self.pcd_path = pcd_path
        self.got_cloud = False

        self.get_logger().info("等待点云数据...")

    def resolve_package_path(self, path):
        """解析路径中的 $(find package) 语法"""
        # 使用正则表达式匹配 $(find package_name)
        pattern = r"\$\(find\s+([a-zA-Z0-9_]+)\)"
        match = re.search(pattern, path)

        if match:
            package_name = match.group(1)
            try:
                # 获取包的共享目录
                package_share_dir = get_package_share_directory(package_name)
                # 替换路径中的 $(find package) 部分
                resolved_path = path.replace(
                    f"$(find {package_name})", package_share_dir
                )
                self.get_logger().info(f"解析路径: {path} -> {resolved_path}")
                return resolved_path
            except Exception as e:
                self.get_logger().warning(f'无法找到包 "{package_name}": {str(e)}')
                # 如果找不到包，尝试使用相对路径
                return path
        return path

    def callback(self, msg):
        """收到点云数据时的回调函数"""
        if self.got_cloud:
            return

        self.get_logger().info("收到点云，正在保存...")

        try:
            # 方法1: 使用read_points_numpy获取numpy数组（推荐）
            try:
                # 尝试获取x, y, z字段
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )

                # 如果points是结构数组，转换为普通数组
                if points.dtype.names:
                    points = np.column_stack(
                        [points[name] for name in points.dtype.names[:3]]
                    )
            except:
                # 方法1失败，尝试方法2
                self.get_logger().info("方法1失败，尝试手动解析点云...")
                points = self.extract_points_manual(msg)

            if points.size == 0 or len(points) == 0:
                self.get_logger().error("点云为空！")
                rclpy.shutdown()
                return

            # 确保points是三维的
            if points.ndim == 1:
                # 可能是结构化数组，尝试重塑
                points = points.reshape(-1, 3)
            elif points.ndim > 2:
                points = points.reshape(-1, 3)

            # 确保形状正确
            if points.shape[1] > 3:
                points = points[:, :3]

            self.get_logger().info(f"点数: {points.shape[0]}")

            # 创建Open3D点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 保存为PCD文件
            o3d.io.write_point_cloud(self.pcd_path, pcd)

            self.get_logger().info(f"点云已保存: {self.pcd_path}")
            self.got_cloud = True

            # 保存完成后关闭节点
            self.get_logger().info("保存完成，关闭节点...")
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"保存失败: {str(e)}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            rclpy.shutdown()

    def extract_points_manual(self, msg):
        """手动解析点云数据（备选方法）"""
        self.get_logger().info("使用手动解析方法...")

        # 使用read_points获取数据
        points_gen = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )

        # 转换为列表
        points_list = []
        for p in points_gen:
            # 确保p是元组或列表，并至少包含3个元素
            if hasattr(p, "__len__") and len(p) >= 3:
                points_list.append([float(p[0]), float(p[1]), float(p[2])])

        if not points_list:
            # 尝试获取所有字段
            points_gen = point_cloud2.read_points(msg, skip_nans=True)
            for p in points_gen:
                # 取前三个字段
                if hasattr(p, "__len__") and len(p) >= 3:
                    points_list.append([float(p[0]), float(p[1]), float(p[2])])

        return np.array(points_list, dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = SavePCDNode()

    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"节点运行错误: {str(e)}")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
