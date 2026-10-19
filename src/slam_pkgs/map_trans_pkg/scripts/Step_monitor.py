#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import time

class StepMonitorNode(Node):
    def __init__(self):
        super().__init__('step_monitor_node')

        # 1. 扩增观测点清单：在这里增加你的坐标
        # x, y: 地图坐标; dist_tol: 触发半径; done: 是否已完成
        self.watch_list = [
            {"id": "A", "x": 1.20, "y": 0.50, "dist_tol": 0.2, "done": False},
            {"id": "B", "x": 1.50, "y": 0.00, "dist_tol": 0.2, "done": False},
            {"id": "C", "x": 1.80, "y": -0.5, "dist_tol": 0.2, "done": False},
            # 你可以在这里继续添加 D, E, F...
        ]

        # 2. 订阅下位机/重定位节点发布的实时位置
        # 注意：请确认你的里程计话题名称，通常是 /Odometry 或 /localization
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)

        # 3. 发布快照触发指令给 step_manager.py
        self.cmd_pub = self.create_publisher(Bool, '/save_snapshot_cmd', 10)

        self.get_logger().info(f"观测点监听器已启动，共加载 {len(self.watch_list)} 个监测点。")

    def odom_callback(self, msg):
        """核心监听逻辑：比对当前坐标与任务点"""
        # 获取当前机器人在 map 系下的坐标
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        for task in self.watch_list:
            if not task["done"]:
                # 计算欧氏距离
                distance = np.sqrt((curr_x - task["x"])**2 + (curr_y - task["y"])**2)

                # 如果进入触发圈
                if distance < task["dist_tol"]:
                    self.get_logger().info(f"检测到机器人到达观测点 {task['id']} (距离: {distance:.2f}m)")
                    self.trigger_snapshot_sequence(task)

    def trigger_snapshot_sequence(self, task):
        """触发快照闭环：开启 -> 识别 -> 关闭锁定"""
        # 1. 发送开启指令
        self.cmd_pub.publish(Bool(data=True))
        self.get_logger().info(f"--- 已开启快照识别 (点位: {task['id']}) ---")

        # 2. 标记任务完成，防止在原地停留时重复触发
        task["done"] = True

        # 3. 延时锁定：给 yolo 和 cube_locator 留出稳定计算的时间
        # 这里使用简单的计时器（实际部署建议用 rclpy.Timer 避免阻塞回调）
        self.create_timer(3.0, lambda: self.stop_snapshot(task['id']), oneshot=True)

    def stop_snapshot(self, task_id):
        """锁定识别数据"""
        self.cmd_pub.publish(Bool(data=False))
        self.get_logger().info(f"--- 已关闭并锁定点位 {task_id} 的识别数据 ---")

def main():
    rclpy.init()
    node = StepMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()