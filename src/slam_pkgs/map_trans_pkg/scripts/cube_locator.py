#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import time
from collections import deque

class CubeLocator(Node):
    def __init__(self):
        super().__init__('cube_locator')

        # 1. 类别字典
        self.class_names = {
            0: 'R_R1', 1: 'B_R1', 2: 'T03', 3: 'T04', 4: 'T05',
            5: 'T06', 6: 'T07', 7: 'T08', 8: 'T09', 9: 'T10',
            10: 'T11', 11: 'T12', 12: 'T13', 13: 'T14', 14: 'T15',
            15: 'T16', 16: 'T17', 17: 'F18', 18: 'F19', 19: 'F20',
            20: 'F21', 21: 'F22', 22: 'F23', 23: 'F24', 24: 'F25',
            25: 'F26', 26: 'F27', 27: 'F28', 28: 'F29', 29: 'F30',
            30: 'F31', 31: 'F32'
        }
        
        # --- 配置：你当前最想关注的目标 ID（设为 None 则打印所有） ---
        self.target_id_to_track = None 

        # 2. 标定参数
        self.K = np.array([[496.232, 0, 302.022], [0, 497.362, 282.194], [0, 0, 1]])
        self.D = np.array([-0.026473, 0.046835, 0.00035, -0.000786])
        self.R = np.array([[ 0.003365, -0.999978,  0.005773],
                           [ 0.078241, -0.005492, -0.996920],
                           [ 0.996929,  0.003806,  0.078220]])
        self.t = np.array([-0.017530, -0.126681, -0.062143])

        # 3. 状态变量与滤波缓存
        self.latest_bboxes = []
        self.history = {}
        self.smooth_size = 8
        self.last_time = time.time()
        self.frame_count = 0

        # 4. 订阅与发布
        self.bbox_sub = self.create_subscription(Float32MultiArray, '/yolo_bboxes', self.bbox_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_callback, 10)
        self.pose_pub = self.create_publisher(Float32MultiArray, '/detected_cube_pose', 10)
        
        self.get_logger().info("优化版融合定位节点已启动，频率统计已修正。")

    def bbox_callback(self, msg):
        data = msg.data
        self.latest_bboxes = [data[i:i+6] for i in range(0, len(data), 6)]

    def lidar_callback(self, msg):
        if not self.latest_bboxes:
            return

        # --- 频率统计移至此处：统计雷达回调的触发频率 ---
        self.frame_count += 1
        now = time.time()
        if now - self.last_time >= 1.0:
            hz = self.frame_count / (now - self.last_time)
            self.get_logger().info(f"== 实时传感器处理帧率: {hz:.2f} FPS ==")
            self.frame_count = 0
            self.last_time = now

        # 提取点云
        pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_list = [[p[0], p[1], p[2]] for p in pts_gen]
        if not pts_list: return
        pts_3d = np.array(pts_list, dtype=np.float32)
        
        # 空间过滤
        mask_spatial = (pts_3d[:, 0] > 0.3) & (pts_3d[:, 0] < 5.0)
        pts_3d = pts_3d[mask_spatial]
        if pts_3d.size == 0: return

        # 向量化投影
        img_pts, _ = cv2.projectPoints(pts_3d, self.R, self.t, self.K, self.D)
        u, v = img_pts[:, 0, 0], img_pts[:, 0, 1]
        
        for box in self.latest_bboxes:
            xc, yc, w, h, conf, cls = box
            cls_id = int(cls)
            l, r, t, b = xc - w/2, xc + w/2, yc - h/2, yc + h/2

            # 筛选点云
            mask_box = (u >= l) & (u <= r) & (v >= t) & (v <= b)
            box_points = pts_3d[mask_box]

            if len(box_points) > 5:
                # 计算坐标
                raw_pos = np.median(box_points, axis=0)
                
                # 滑动平均平滑
                if cls_id not in self.history:
                    self.history[cls_id] = deque(maxlen=self.smooth_size)
                self.history[cls_id].append(raw_pos)
                smooth_pos = np.mean(self.history[cls_id], axis=0)

                # 发布坐标话题
                pub_msg = Float32MultiArray()
                pub_msg.data = [float(cls_id), float(smooth_pos[0]), float(smooth_pos[1]), float(smooth_pos[2]), float(conf)]
                self.pose_pub.publish(pub_msg)

                # --- 智能打印逻辑 ---
                # 只有当 ID 匹配或未设置 track_id 时才打印，防止刷屏
                if self.target_id_to_track is None or cls_id == self.target_id_to_track:
                    label = self.class_names.get(cls_id, f"Unknown({cls_id})")
                    print(f"[锁定] {label}({cls_id}) | X:{smooth_pos[0]:.3f} Y:{smooth_pos[1]:.3f} Z:{smooth_pos[2]:.3f}")

def main():
    rclpy.init()
    node = CubeLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()