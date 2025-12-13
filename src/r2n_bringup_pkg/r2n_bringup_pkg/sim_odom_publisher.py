#!/usr/bin/env python3
# 替代 amcl 功能

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


class SimOdomPublisher(Node):
    def __init__(self):
        super().__init__("sim_odom_publisher")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 5)

        self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(
            "SimOdomPublisher 已启动：直接转发 world→base_link 为 odom→body（完美定位模式）"
        )

    def broadcast_map_to_odom(self, stamp):
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "world",
                "base_link",
                rclpy.time.Time(),  # 仍然查最新
            )
        except TransformException as ex:
            # self.get_logger().warn(f"无法获取 world → base_link: {ex}")
            return

        stamp = trans.header.stamp
        self.broadcast_map_to_odom(stamp)

        t_odom_to_body = TransformStamped()
        t_odom_to_body.header.stamp = stamp
        t_odom_to_body.header.frame_id = "odom"  # 改名
        t_odom_to_body.child_frame_id = "body"  # 改名
        t_odom_to_body.transform = trans.transform  # 完全原样复制！不取反！不逆旋转！

        self.tf_broadcaster.sendTransform(t_odom_to_body)

        # 发布 odometry（完美里程计）
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = trans.transform.translation.x
        odom.pose.pose.position.y = trans.transform.translation.y
        odom.pose.pose.position.z = trans.transform.translation.z
        odom.pose.pose.orientation = trans.transform.rotation

        # 极小协方差 = 完美定位
        cov = [1e-9] * 36
        cov[0] = cov[7] = cov[14] = 1e-6
        cov[35] = 1e-6
        odom.pose.covariance = cov

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
