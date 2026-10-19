#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2
from scipy.spatial.distance import cdist

class ClusterTracker:
    """聚类跟踪器：通过增强平滑和容错记忆解决抖动与闪现 ID"""
    def __init__(self, max_distance=1.0, min_lifetime=10, max_age=30): # 增加 max_age 到 15 帧
        self.clusters = []
        self.max_distance = max_distance
        self.min_lifetime = min_lifetime
        self.max_age = max_age # 物体消失后最多保留 15 帧记忆
        self.next_id = 0
        
    def update(self, new_clusters):
        if not self.clusters:
            for cluster in new_clusters:
                self.clusters.append({
                    'id': self.next_id,
                    'center': cluster['center'],
                    'bbox': cluster['bbox'],
                    'size_pts': cluster['size_pts'],
                    'age': 0,
                    'matched_count': 1,
                    'is_lost': False,
                    'history': [cluster]
                })
                self.next_id += 1
            return self.get_stable_clusters()
        
        if new_clusters:
            old_centers = np.array([c['center'] for c in self.clusters])
            new_centers = np.array([c['center'] for c in new_clusters])
            distances = cdist(old_centers, new_centers)
            
            matched_pairs = []
            # 贪婪匹配优化
            for i in range(len(self.clusters)):
                if len(new_clusters) > 0:
                    min_dist_idx = np.argmin(distances[i])
                    if distances[i][min_dist_idx] < self.max_distance:
                        matched_pairs.append((i, min_dist_idx))
                        distances[:, min_dist_idx] = np.inf # 防止多个旧目标匹配同一个新目标
            
            matched_new_indices = set()
            for old_idx, new_idx in matched_pairs:
                matched_new_indices.add(new_idx)
                target = self.clusters[old_idx]
                target['matched_count'] += 1
                target['age'] = 0
                target['is_lost'] = False
                
                # --- [优化] 增强平滑滤波，解决抖动 ---
                # alpha 越小越丝滑，但延迟越高。设为 0.2 以应对剧烈抖动
                alpha = 0.2 
                new_c = new_clusters[new_idx]
                target['center'] = alpha * new_c['center'] + (1 - alpha) * target['center']
                target['size_pts'] = int(alpha * new_c['size_pts'] + (1 - alpha) * target['size_pts'])
                
                # 对包围盒尺寸也进行平滑
                old_bbox = target['bbox']
                new_bbox = new_c['bbox']
                old_bbox['min'] = alpha * new_bbox['min'] + (1 - alpha) * old_bbox['min']
                old_bbox['max'] = alpha * new_bbox['max'] + (1 - alpha) * old_bbox['max']
                old_bbox['size'] = old_bbox['max'] - old_bbox['min']
            
            # 未匹配的新物体
            for i, cluster in enumerate(new_clusters):
                if i not in matched_new_indices:
                    self.clusters.append({
                        'id': self.next_id,
                        'center': cluster['center'],
                        'bbox': cluster['bbox'],
                        'size_pts': cluster['size_pts'],
                        'age': 0,
                        'matched_count': 1,
                        'is_lost': False,
                        'history': [cluster]
                    })
                    self.next_id += 1
        
        # 生命周期管理
        for cluster in self.clusters:
            cluster['age'] += 1
            if cluster['age'] > 1: # 如果本帧没匹配上
                cluster['is_lost'] = True
        
        # 移除超龄物体
        self.clusters = [c for c in self.clusters if c['age'] < self.max_age]
        return self.get_stable_clusters()
    
    def get_stable_clusters(self):
        # 仅返回已经稳定存在一段时间且当前未丢失的目标
        return [c for c in self.clusters if c['matched_count'] >= self.min_lifetime and not c['is_lost']]

class PointCloudClusteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_clustering_node')
        
        # 参数调整：减小 voxel_size 以获取更多细节，减小抖动
        self.declare_parameter('eps', 0.1)
        self.declare_parameter('min_points', 8)
        self.declare_parameter('max_points', 1200)
        self.declare_parameter('min_cluster_size', 20) # 降低门槛，防止闪现消失
        self.declare_parameter('voxel_size', 0.01)     # 提高精细度至 1cm
        self.declare_parameter('z_min', -0.4)
        self.declare_parameter('z_max', 0.8)
        self.declare_parameter('radius_filter', 3.5)

        self.eps = self.get_parameter('eps').value
        self.min_points = self.get_parameter('min_points').value
        self.max_points = self.get_parameter('max_points').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.radius_filter = self.get_parameter('radius_filter').value
        
        # 容错配置优化：max_age 设为 15，允许约 1.5 秒的数据缺失而不丢 ID
        self.tracker = ClusterTracker(max_distance=0.8, min_lifetime=5, max_age=15)
        
        self.pc_subscriber = self.create_subscription(
            PointCloud2, '/cloud_registered', self.pointcloud_callback, 10)
        
        self.cluster_publisher = self.create_publisher(MarkerArray, '/pointcloud_clusters', 10)
        self.filtered_pc_publisher = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)
        
        self.get_logger().info("方块识别增强版启动：已针对抖动和 ID 闪现进行优化")

    def pointcloud_callback(self, msg):
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            if points.shape[0] == 0: return
            
            raw_clusters = self.process_pointcloud(points)
            stable_clusters = self.tracker.update(raw_clusters)
            
            if stable_clusters:
                self.publish_clusters(stable_clusters, msg.header)
        except Exception as e:
            self.get_logger().error(f"处理错误: {str(e)}")

    def process_pointcloud(self, points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        pts = np.asarray(pcd.points)
        dist_mask = np.linalg.norm(pts, axis=1) < self.radius_filter
        pcd = pcd.select_by_index(np.where(dist_mask)[0])
        
        pts = np.asarray(pcd.points)
        if len(pts) > 0:
            angles = np.arctan2(pts[:, 1], pts[:, 0])
            angle_mask = (angles >= -np.pi/3) & (angles <= np.pi/3) & (pts[:, 0] > 0)
            pcd = pcd.select_by_index(np.where(angle_mask)[0])
        
        pts = np.asarray(pcd.points)
        if len(pts) == 0: return []
        z_mask = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        pcd = pcd.select_by_index(np.where(z_mask)[0])
        
        if len(pcd.points) < self.min_points: return []
        
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        self.publish_filtered_pointcloud(pcd, msg_frame="body")
        
        labels = np.array(pcd.cluster_dbscan(eps=self.eps, min_points=self.min_points))
        
        clusters = []
        for i in range(labels.max() + 1):
            indices = np.where(labels == i)[0]
            if self.min_cluster_size <= len(indices) <= self.max_points:
                cluster_pts = np.asarray(pcd.points)[indices]
                bbox = self.compute_stable_bounding_box(cluster_pts)
                
                size = bbox['size']
                if size[1] > 0.75: continue # 宽度过滤
                if size[0] < 0.08 or size[2] < 0.08: continue # 噪声过滤
                
                clusters.append({
                    'center': np.mean(cluster_pts, axis=0),
                    'size_pts': len(indices),
                    'bbox': bbox
                })
        return clusters

    def compute_stable_bounding_box(self, points):
        p = 8 # 增加分位数，进一步收缩边界以过滤边缘噪点
        min_b = np.percentile(points, p, axis=0)
        max_b = np.percentile(points, 100-p, axis=0)
        return {'min': min_b, 'max': max_b, 'size': max_b - min_b}

    def publish_filtered_pointcloud(self, pcd, msg_frame):
        pts = np.asarray(pcd.points)
        if len(pts) == 0: return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg_frame)
        msg = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        self.filtered_pc_publisher.publish(msg)

    def publish_clusters(self, clusters, header):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for c in clusters:
            m = Marker(header=header)
            m.header.stamp, m.ns, m.id = now, "clusters", int(c['id'])
            m.type, m.action = Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, c['center'])
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = map(float, c['bbox']['size'])
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.5 # 绿色更醒目
            m.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(m)
            
            t = Marker(header=header)
            t.header.stamp, t.ns, t.id = now, "text", int(c['id'] + 1000)
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(c['center'][0]), float(c['center'][1]), float(c['center'][2] + 0.4)
            t.scale.z, t.color.a = 0.15, 1.0
            t.color.r, t.color.g, t.color.b = 1.0, 1.0, 1.0
            t.text = f"ID:{c['id']} Stable"
            marker_array.markers.append(t)
        self.cluster_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()