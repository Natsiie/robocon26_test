#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion

class TransformFusion(Node):
    def __init__(self):
        super().__init__('transform_fusion_node')
        
        # 1. 订阅定位结果 (低频，约 0.5Hz)
        self.sub = self.create_subscription(
            Odometry,
            '/map_to_odom', 
            self.callback,
            10
        )
        
        self.br = TransformBroadcaster(self)
        
        # 2. 内部缓存最新的位姿 (默认为单位阵，即原点)
        self.current_transform = TransformStamped()
        self.current_transform.header.frame_id = 'map'
        self.current_transform.child_frame_id = 'odom'
        self.current_transform.transform.rotation.w = 1.0
        
        # 3. 启动高频定时器 (50Hz)，确保持续广播 TF
        self.timer = self.create_timer(0.02, self.publish_tf_loop)
        
        self.get_logger().info("Transform Fusion Node Started (High Freq Mode)")

    def callback(self, msg):
        # 收到低频的定位修正消息，更新缓存
        self.current_transform.transform.translation.x = msg.pose.pose.position.x
        self.current_transform.transform.translation.y = msg.pose.pose.position.y
        self.current_transform.transform.translation.z = msg.pose.pose.position.z
        self.current_transform.transform.rotation = msg.pose.pose.orientation
        # 注意：这里只更新数据，不直接发，由定时器统一发

    def publish_tf_loop(self):
        # 每次发送时更新时间戳为当前时间
        self.current_transform.header.stamp = self.get_clock().now().to_msg()
        # 广播 TF
        self.br.sendTransform(self.current_transform)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()