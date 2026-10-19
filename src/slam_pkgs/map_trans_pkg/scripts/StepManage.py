#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class StepManager(Node):
    def __init__(self):
        super().__init__('step_manager')

        # 1. 初始化 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2. 12个台阶的 2D 理论坐标
        # 只要 X, Y 匹配上了，电控就会根据 ID 执行对应的高度动作
        self.step_theoretical_poses = {
            1: [1.0, 0.6], 2: [1.0, 0.0], 3: [1.0, -0.6], 
            4: [1.2, 0.6], 5: [1.2, 0.0], 6: [1.2, -0.6], 
            7: [1.4, 0.6], 8: [1.4, 0.0], 9: [1.4, -0.6], 
            10:[1.6, 0.6], 11:[1.6, 0.0], 12:[1.6, -0.6], 
        }

        # 3. 内存数据库
        self.step_database = {i: {"is_occupied": False, "cube_id": -1, "pos": None} for i in range(1, 13)}

        # 4. 订阅与发布
        self.pose_sub = self.create_subscription(Float32MultiArray, '/detected_cube_pose', self.cube_callback, 10)
        self.cmd_sub = self.create_subscription(Bool, '/save_snapshot_cmd', self.command_callback, 10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/step_markers', 10)
        self.mission_pub = self.create_publisher(Float32MultiArray, '/final_step_mission', 10)

        self.is_scanning = False
        self.get_logger().info(">>> 台阶管理器已启动。监听 map -> body 变换。")

    def command_callback(self, msg):
        self.is_scanning = msg.data
        if not self.is_scanning:
            self.publish_final_mission()

    def cube_callback(self, msg):
        if not self.is_scanning:
            return

        data = msg.data
        cls_id = int(data[0])
        local_pos = [data[1], data[2], data[3]] # 这里的坐标是相对于 body 的

        # 坐标转换：body -> map
        global_pos = self.get_global_pose(local_pos)
        if global_pos is None: return

        # 2D 匹配逻辑
        step_id = self.match_to_step_2d(global_pos)
        
        if step_id:
            self.step_database[step_id]["is_occupied"] = True
            self.step_database[step_id]["cube_id"] = cls_id
            self.step_database[step_id]["pos"] = global_pos 
            self.get_logger().info(f"锁定：方块 {cls_id} -> 台阶 {step_id}")
            self.publish_rviz_markers()

    def get_global_pose(self, local_pos):
        try:
            # 关键：使用 relocalization.cpp 中定义的 'body' 系
            transform = self.tf_buffer.lookup_transform('map', 'body', rclpy.time.Time())
            p = PointStamped()
            p.header.frame_id = 'body'
            p.point.x, p.point.y, p.point.z = local_pos
            p_map = tf2_geometry_msgs.do_transform_point(p, transform)
            return [p_map.point.x, p_map.point.y, p_map.point.z]
        except Exception as e:
            return None

    def match_to_step_2d(self, global_pos):
        best_step_id = None
        min_dist = 0.15 
        detected_xy = np.array([global_pos[0], global_pos[1]])

        for s_id, s_xy in self.step_theoretical_poses.items():
            dist = np.linalg.norm(detected_xy - np.array(s_xy))
            if dist < min_dist:
                min_dist = dist
                best_step_id = s_id
        return best_step_id

    def publish_final_mission(self):
        mission_msg = Float32MultiArray()
        mission_msg.data = [float(self.step_database[i]["cube_id"]) for i in range(1, 13)]
        self.mission_pub.publish(mission_msg)

    def publish_rviz_markers(self):
        markers = MarkerArray()
        for i, info in self.step_database.items():
            if not info["is_occupied"]: continue
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = info["pos"]
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 1.0, 0.0, 1.0)
            markers.markers.append(m)
        self.marker_pub.publish(markers)

def main():
    rclpy.init()
    node = StepManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()