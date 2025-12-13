#!/usr/bin/env python3

import yaml
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__("map_publisher_node")

        self.declare_parameter("map_path", "")
        self.declare_parameter("publish_topic", "/map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_interval", 5.0)

        yaml_path = Path(self.get_parameter("map_path").value).expanduser().resolve()
        publish_topic = self.get_parameter("publish_topic").value
        frame_id = self.get_parameter("frame_id").value
        publish_interval = self.get_parameter("publish_interval").value

        if not yaml_path.exists():
            raise FileNotFoundError(f"Map YAML not found: {yaml_path}")
        s = "Reading map YAML: {yaml_path}"
        self.get_logger().info(s.format(yaml_path=yaml_path))

        with open(yaml_path, "r") as f:
            map_meta = yaml.safe_load(f)

        # === 关键：完全按照 map_server 的方式解析 image 路径 ===
        image_path_str = map_meta["image"]
        if image_path_str.startswith("package://"):
            parts = image_path_str[10:].split("/", 1)
            pkg_name = parts[0]
            rel_path = parts[1] if len(parts) > 1 else ""
            image_path = Path(get_package_share_directory(pkg_name)) / rel_path
        else:
            image_path = (yaml_path.parent / image_path_str).resolve()

        if not image_path.exists():
            raise FileNotFoundError(f"Map image not found: {image_path}")

        # === 关键：直接读取为灰度图，不要做任何 flip 或 反转！===
        img = Image.open(image_path).convert("L")
        data = np.array(img, dtype=np.uint8)

        # === 完全按照官方 map_server 的阈值转换逻辑 ===
        occdata = np.full_like(data, -1, dtype=np.int8)
        occdata[data <= 10] = 100  # 黑色（占用）
        occdata[data >= 240] = 0  # 白色（空闲）
        # 10 < data < 240 保持 -1（未知），205 灰色就是 -1

        # 如果 yaml 中指定了 occupied_thresh 和 free_thresh，则使用它们
        occupied_thresh = map_meta.get("occupied_thresh", 0.65)
        free_thresh = map_meta.get("free_thresh", 0.196)
        p_occupied = 1.0 - occupied_thresh
        p_free = free_thresh

        # 官方转换公式（参考 map_server 源码）
        occdata = np.full_like(data, -1, dtype=np.int8)
        occdata[data < p_occupied * 255] = 100  # 比 occupied_thresh 更黑 → 占用
        occdata[data > p_free * 255] = 0  # 比 free_thresh 更白 → 空闲

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id

        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = float(map_meta["resolution"])
        msg.info.width = data.shape[1]
        msg.info.height = data.shape[0]

        origin = map_meta.get("origin", [0.0, 0.0, 0.0])
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        occdata = np.flipud(occdata)
        msg.data = occdata.flatten().tolist()

        self.msg = msg
        self.pub = self.create_publisher(
            OccupancyGrid,
            publish_topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.timer = self.create_timer(publish_interval, self.publish)

        self.publish()
        self.get_logger().info(
            f"Map loaded successfully: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m"
        )
        self.get_logger().info(
            f"Origin: ({origin[0]}, {origin[1]}) → publishing on {publish_topic}"
        )

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
