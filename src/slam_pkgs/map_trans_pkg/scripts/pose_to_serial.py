#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import serial
import struct
# [修改] 删除了 import math

class PoseToSerialNode(Node):
    def __init__(self):
        super().__init__('pose_to_serial_node')

        # ================= 配置区域 =================
        self.port_name = '/dev/ttyUSB0' # 请确认端口
        self.baud_rate = 115200
        
        # 目标点
        self.target_x = 0.08
        self.target_y = 2.43  
        # [修改] 删除了 goal_tolerance 阈值参数
        # ===========================================

        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f" 串口已连接: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f" 串口打开失败: {e}")
            self.serial_port = None

        self.loc_sub = self.create_subscription(
            Odometry, 
            '/localization', 
            self.loc_callback, 
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info(f"当前目标点: [{self.target_x}, {self.target_y}]")

    def goal_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.get_logger().info(f" 目标点更新: [{self.target_x:.2f}, {self.target_y:.2f}]")

    def loc_callback(self, msg):
        if self.serial_port is None or not self.serial_port.is_open:
            return

        # 1. 获取当前数据
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # [修改] 删除了距离计算和 stop_flag 逻辑

        # ---------------------------------------------------------
        # 2. 数据打包 (16字节: 4个float)
        # ---------------------------------------------------------
        # [关键修改] 格式改回 '<ffff'，去掉了后面的 B
        data_bytes = struct.pack('<ffff', current_x, current_y, self.target_x, self.target_y)

        # 3. 帧头 (2字节)
        header = b'\xFF\xFE'

        # 4. 计算校验和 (1字节)
        # 这里的 sum(data_bytes) 会自动计算 16 字节的和，逻辑不用变
        checksum_value = sum(header) + sum(data_bytes)
        checksum = struct.pack('B', checksum_value & 0xFF)

        # 5. 帧尾 (2字节)
        tail = b'\xFD\xFC'

        # 6. 拼接最终包 (总长恢复为 21 字节)
        packet = header + data_bytes + checksum + tail

        # 7. 发送
        try:
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f"串口发送异常: {e}")

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()