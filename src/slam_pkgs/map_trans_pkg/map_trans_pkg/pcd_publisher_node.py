#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from pcl_msgs.msg import PointIndices  # 不需要，但保留兼容
from ament_index_python.packages import get_package_share_directory


def o3d_to_ros(
    o3d_pc: o3d.geometry.PointCloud, frame_id: str, stamp=None
) -> PointCloud2:
    import sensor_msgs_py.point_cloud2 as pc2
    from std_msgs.msg import Header

    points = np.asarray(o3d_pc.points, dtype=np.float32)
    if points.size == 0:
        raise ValueError("Point cloud is empty!")

    header = Header()
    if stamp is not None:
        header.stamp = stamp
    else:
        # 注意：这里 stamp 留空，由调用方在节点内填充当前时间
        header.stamp.sec = 0
        header.stamp.nanosec = 0
    header.frame_id = frame_id

    fields = [
        pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
    ]

    cloud_msg = pc2.create_cloud(header, fields, points)
    return cloud_msg


class PCDPublisherNode(Node):
    def __init__(self):
        super().__init__("pcd_publish_node")

        # 参数声明（带默认值）
        self.declare_parameter("pcd_path", "$(find r2n_bringup_pkg)/pcds/scan.pcd")
        self.declare_parameter("publish_topic", "/map_3d")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_interval", 20.0)

        # 读取参数
        raw_pcd_path = self.get_parameter("pcd_path").get_parameter_value().string_value
        self.publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.publish_interval = (
            self.get_parameter("publish_interval").get_parameter_value().double_value
        )

        # 解析 $(find xxx_pkg) 路径
        if raw_pcd_path.startswith("$(find "):
            try:
                # 提取包名
                pkg_name = raw_pcd_path.split("$(find ")[1].split(")")[0]
                share_dir = get_package_share_directory(pkg_name)
                rel_path = raw_pcd_path.split(")")[1].lstrip("/")
                self.pcd_path = os.path.join(share_dir, rel_path)
                self.get_logger().info(f"Resolved PCD path: {self.pcd_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to resolve package path: {e}")
                raise
        else:
            self.pcd_path = os.path.expanduser(raw_pcd_path)

        # 检查文件是否存在
        if not os.path.isfile(self.pcd_path):
            self.get_logger().error(f"PCD file not found: {self.pcd_path}")
            raise FileNotFoundError(self.pcd_path)

        # 加载 PCD
        self.get_logger().info(f"Loading PCD file: {self.pcd_path}")
        o3d_cloud = o3d.io.read_point_cloud(self.pcd_path)
        if o3d_cloud.is_empty():
            self.get_logger().error("Loaded PCD is empty!")
            raise ValueError("Empty point cloud")
        self.get_logger().info(f"Loaded {len(o3d_cloud.points)} points")

        # 创建发布者
        self.publisher_ = self.create_publisher(PointCloud2, self.publish_topic, 10)

        # 定时器
        if self.publish_interval > 0.0:
            period = self.publish_interval
        else:
            period = 20.0
            self.get_logger().warn("publish_interval <= 0, set to 20.0s")

        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        o3d_cloud = o3d.io.read_point_cloud(self.pcd_path)
        if o3d_cloud.is_empty():
            self.get_logger().warn("Loaded PCD is empty, skipping publish")
            return

        current_stamp = self.get_clock().now().to_msg()
        msg = o3d_to_ros(o3d_cloud, self.frame_id, stamp=current_stamp)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published map point cloud to {self.publish_topic} "
            f"(frame_id: {self.frame_id}, points: {len(o3d_cloud.points)})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Exception in node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
