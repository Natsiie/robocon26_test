#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from sklearn.linear_model import RANSACRegressor

class GridStatusManager(Node):
    def __init__(self):
        super().__init__('grid_status_manager')
        self.class_names = {i: f"T{i:02d}" for i in range(32)}
        self.class_names.update({0: 'R_R1', 1: 'B_R1'})

        # 动态边界滤波
        self.smooth_bbox = None
        self.alpha = 0.2
        self.ransac = RANSACRegressor(residual_threshold=0.03)

        # 相机参数
        self.K = np.array([[496.232, 0, 302.022], [0, 497.362, 282.194], [0, 0, 1]])
        self.D = np.array([-0.026473, 0.046835, 0.00035, -0.000786])
        self.t_front = np.array([0.15, 0.0, 0.1])

        self.pc_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.process_cloud, 10)
        self.yolo_sub = self.create_subscription(Float32MultiArray, '/yolo_bboxes_front', self.yolo_cb, 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/grid_item_status', 10)
        self.filtered_pc_pub = self.create_publisher(PointCloud2, '/filtered_shelf_pc', 10)

        self.latest_bboxes = []
        self.get_logger().info("九宫格序号校准版启动：1-3底, 4-6中, 7-9顶")

    def yolo_cb(self, msg):
        data = msg.data
        self.latest_bboxes = [data[i:i+6] for i in range(0, len(data), 6)]

    def process_cloud(self, msg):
        pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_3d = np.array([list(p) for p in list(pts_gen)], dtype=np.float32)
        if pts_3d.size == 0: return

        # --- 核心修正：适配你当前 Z: 0.05~1.68 的坐标系 ---
        azimuths = np.arctan2(pts_3d[:, 1], pts_3d[:, 0]) * 180.0 / np.pi
        mask = (azimuths >= -45) & (azimuths <= 45) & \
               (pts_3d[:, 2] > 0.05) & (pts_3d[:, 2] < 2.8) & \
               (pts_3d[:, 0] > 0.5) & (pts_3d[:, 0] < 3.0)
        shelf_pts = pts_3d[mask]
        
        if shelf_pts.shape[0] < 30: return

        # 发布点云并在终端打印调试信息，解决 RViz 看不到的问题
        self.filtered_pc_pub.publish(pc2.create_cloud_xyz32(msg.header, shelf_pts.tolist()))
        # self.get_logger().info(f"正在向 RViz 发布点云: {len(shelf_pts)} 点, Frame: {msg.header.frame_id}")

        try:
            self.ransac.fit(shelf_pts[:, [1, 2]], shelf_pts[:, 0])
            inliers = shelf_pts[self.ransac.inlier_mask_]
            raw_bbox = np.array([np.min(inliers[:, 1]), np.max(inliers[:, 1]), np.min(inliers[:, 2]), np.max(inliers[:, 2])])

            if self.smooth_bbox is None: self.smooth_bbox = raw_bbox
            else: self.smooth_bbox = self.smooth_bbox * (1 - self.alpha) + raw_bbox * self.alpha
            
            m_y_min, m_y_max, m_z_min, m_z_max = self.smooth_bbox
            step_y = (m_y_max - m_y_min) / 3.0
            step_z = (m_z_max - m_z_min) / 3.0
        except: return

        grid_results, grid_counts = [], []
        # row 0(底)->2(顶), col 0(右/左)->2
        for row in range(3):
            for col in range(3):
                c_y_min = m_y_min + col * step_y
                c_y_max = m_y_min + (col + 1) * step_y
                c_z_min = m_z_min + row * step_z
                c_z_max = m_z_min + (row + 1) * step_z
                
                t_x = self.ransac.predict(np.array([[(c_y_min+c_y_max)/2, (c_z_min+c_z_max)/2]]))[0]
                cell_mask = (pts_3d[:, 1] > c_y_min + 0.03) & (pts_3d[:, 1] < c_y_max - 0.03) & \
                            (pts_3d[:, 2] > c_z_min + 0.03) & (pts_3d[:, 2] < c_z_max - 0.03) & \
                            (np.abs(pts_3d[:, 0] - t_x) < 0.25)
                
                count = pts_3d[cell_mask].shape[0]
                grid_counts.append(count)
                grid_results.append(self.identify_item_class(np.mean(pts_3d[cell_mask], axis=0)) if count > 20 else -1)

        self.display_grid(grid_results, grid_counts, self.smooth_bbox)
        self.status_pub.publish(Int32MultiArray(data=grid_results))

    def display_grid(self, results, counts, bbox):
        print("\n" + "="*60)
        print(f"动态边界 [Y:{bbox[0]:.2f}~{bbox[1]:.2f}] [Z:{bbox[2]:.2f}~{bbox[3]:.2f}]")
        res_m = [results[i:i+3] for i in range(0, 9, 3)]
        cnt_m = [counts[i:i+3] for i in range(0, 9, 3)]
        # 终端显示：顶行在上，视觉对齐
        for r_idx, (r_data, c_data) in enumerate(zip(reversed(res_m), reversed(cnt_m))):
            row_res = [self.class_names.get(id, "EMPTY" if id == -1 else "UNKNOWN") for id in r_data]
            # 打印逻辑行号 7-9, 4-6, 1-3
            row_label = 7 - r_idx * 3
            print(f"L{row_label}-{row_label+2} | {row_res[0]:^8} | {row_res[1]:^8} | {row_res[2]:^8} |   | {c_data[0]:^4} | {c_data[1]:^4} | {c_data[2]:^4} |")
        print("="*60)

    def identify_item_class(self, pos_3d):
        img_pts, _ = cv2.projectPoints(np.array([pos_3d]), np.eye(3), self.t_front, self.K, self.D)
        u, v = img_pts[0, 0]
        for box in self.latest_bboxes:
            xc, yc, w, h, conf, cls_idx = box
            if (xc - w/2 < u < xc + w/2) and (yc - h/2 < v < yc + h/2): return int(cls_idx)
        return -2 

def main():
    rclpy.init(); rclpy.spin(GridStatusManager()); rclpy.shutdown()

if __name__ == '__main__': main()