#!/usr/bin/env python3
# coding=utf8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import copy
import time
import numpy as np
import open3d as o3d
import math

# ROS 2 消息
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
# [关键修复] 补上这个漏掉的 Header
from std_msgs.msg import Header 
import sensor_msgs_py.point_cloud2 as pc2

# ==============================================================================
# 手写数学库
# ==============================================================================
def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    _2x = x + x; _2y = y + y; _2z = z + z
    _2xw = _2x * w; _2yw = _2y * w; _2zw = _2z * w
    _2xx = _2x * x; _2xy = _2x * y; _2xz = _2x * z
    _2yy = _2y * y; _2yz = _2y * z; _2zz = _2z * z

    M = np.eye(4)
    M[0, 0] = 1.0 - (_2yy + _2zz)
    M[0, 1] = _2xy - _2zw
    M[0, 2] = _2xz + _2yw
    M[1, 0] = _2xy + _2zw
    M[1, 1] = 1.0 - (_2xx + _2zz)
    M[1, 2] = _2yz - _2xw
    M[2, 0] = _2xz - _2yw
    M[2, 1] = _2yz + _2xw
    M[2, 2] = 1.0 - (_2xx + _2yy)
    return M

def translation_matrix(direction):
    M = np.eye(4)
    M[:3, 3] = direction[:3]
    return M

def quaternion_from_matrix(matrix):
    m = matrix[:3, :3]
    tr = np.trace(m)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m[2, 1] - m[1, 2]) / S
        y = (m[0, 2] - m[2, 0]) / S
        z = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / S
        x = 0.25 * S
        y = (m[0, 1] + m[1, 0]) / S
        z = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / S
        x = (m[0, 1] + m[1, 0]) / S
        y = 0.25 * S
        z = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / S
        x = (m[0, 2] + m[2, 0]) / S
        y = (m[1, 2] + m[2, 1]) / S
        z = 0.25 * S
    return [x, y, z, w]

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')
        
        self.get_logger().info('>>> VERSION: FIXED_HEADER_IMPORT <<<')
        self.get_logger().info('Localization Node Inited (ROS 2 Python Version)...')

        self.MAP_VOXEL_SIZE = 0.4
        self.SCAN_VOXEL_SIZE = 0.1
        self.FREQ_LOCALIZATION = 0.5 
        self.LOCALIZATION_TH = 0.7 # 保持 0.6 比较容易匹配成功
        self.FOV = 6.28
        self.FOV_FAR = 30

        self.global_map = None
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)

        self.create_subscription(PointCloud2, '/cloud_registered', self.cb_save_cur_scan, qos_profile)
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, qos_profile)
        self.create_subscription(PointCloud2, '/map', self.cb_global_map, qos_reliable)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.cb_initial_pose, qos_reliable)

        self.timer = self.create_timer(1.0 / self.FREQ_LOCALIZATION, self.thread_localization)
        self.get_logger().warn('Waiting for global map......')

    def msg_to_array(self, msg):
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        dat = [[float(p[0]), float(p[1]), float(p[2])] for p in gen]
        if not dat:
            return np.empty((0, 3), dtype=np.float64)
        return np.array(dat, dtype=np.float64)

    def cb_global_map(self, msg):
        if self.global_map is not None: return 
        self.get_logger().info('Received global map message.')
        
        pc_data = self.msg_to_array(msg)
        if len(pc_data) == 0:
            self.get_logger().warn('Received empty global map!')
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_data)
        self.global_map = pcd.voxel_down_sample(self.MAP_VOXEL_SIZE)
        self.get_logger().info(f'Global map loaded. Points: {len(self.global_map.points)}')

    def cb_save_cur_scan(self, msg):
        # 调试转发
        debug_msg = msg
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = 'camera_init'
        self.pub_pc_in_map.publish(debug_msg)

        # 转换数据
        pc_data = self.msg_to_array(msg)
        if len(pc_data) == 0: return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_data)
        self.cur_scan = pcd

    def cb_initial_pose(self, msg):
        self.get_logger().info('Received initial pose.')
        initial_pose = self.pose_to_mat(msg.pose.pose)
        if self.global_map is None:
            self.get_logger().warn('Global map not loaded yet.')
            return
        if self.cur_scan is None:
            self.get_logger().warn('First scan not received.')
            return
        
        # 这里会调用 run_global_localization，之前就是在里面报错的
        if self.run_global_localization(initial_pose):
            self.initialized = True
            self.get_logger().info('Initialize successfully!!!!!!')

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg

    def thread_localization(self):
        if not self.initialized:
            if self.global_map is None:
                self.get_logger().warn('Waiting for global map...', throttle_duration_sec=2.0)
            else:
                self.get_logger().warn('Waiting for initial pose...', throttle_duration_sec=2.0)
            return
        self.run_global_localization(self.T_map_to_odom)

    def run_global_localization(self, pose_estimation):
        if self.cur_scan is None or self.global_map is None or self.cur_odom is None:
            return False

        self.get_logger().info('Global localization by scan-to-map matching......')
        scan_tobe_mapped = copy.deepcopy(self.cur_scan)
        
        tic = time.time()
        global_map_in_FOV = self.crop_global_map_in_FOV(self.global_map, pose_estimation, self.cur_odom)
        
        if global_map_in_FOV is None or len(global_map_in_FOV.points) < 10:
            self.get_logger().warn('No map points in FOV!')
            return False

        transformation, _ = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
        )
        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1
        )

        toc = time.time()
        self.get_logger().info(f'Time: {toc - tic:.4f}s, Fitness: {fitness:.4f}')

        if fitness > self.LOCALIZATION_TH:
            self.T_map_to_odom = transformation
            map_to_odom_msg = Odometry()
            # 修正坐标计算
            xyz = transformation[:3, 3]
            quat = quaternion_from_matrix(transformation)
            
            map_to_odom_msg.pose.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            map_to_odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            map_to_odom_msg.header.stamp = self.cur_odom.header.stamp
            map_to_odom_msg.header.frame_id = 'map'
            self.pub_map_to_odom.publish(map_to_odom_msg)
            return True
        else:
            self.get_logger().warn('Not match!!!!')
            return False

    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        source = pc_scan.voxel_down_sample(self.SCAN_VOXEL_SIZE * scale)
        target = pc_map.voxel_down_sample(self.MAP_VOXEL_SIZE * scale)
        result_icp = o3d.pipelines.registration.registration_icp(
            source, target, 1.0 * scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return result_icp.transformation, result_icp.fitness

    def crop_global_map_in_FOV(self, global_map, pose_estimation, cur_odom):
        T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        try:
            T_base_link_to_map = np.linalg.inv(T_map_to_base_link)
        except np.linalg.LinAlgError:
            return None

        global_map_points = np.asarray(global_map.points)
        ones = np.ones((len(global_map_points), 1))
        points_hom = np.hstack((global_map_points, ones))
        global_map_in_base_link = np.matmul(T_base_link_to_map, points_hom.T).T[:, :3]

        x = global_map_in_base_link[:, 0]
        y = global_map_in_base_link[:, 1]
        dist_mask = (x > 0) & (x < self.FOV_FAR)
        angle_mask = np.abs(np.arctan2(y, x)) < (self.FOV / 2.0)
        indices = np.where(dist_mask & angle_mask)[0]
        
        if len(indices) == 0: return None

        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(global_map_points[indices])
        
        # [这里] 就是之前报错的地方，现在有 Header 了就不会炸了
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        self.publish_point_cloud(self.pub_submap, header, global_map_points[indices][::5])
        return global_map_in_FOV

    def publish_point_cloud(self, publisher, header, points):
        msg = pc2.create_cloud_xyz32(header, points)
        publisher.publish(msg)

    def pose_to_mat(self, pose):
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        t = [pose.position.x, pose.position.y, pose.position.z]
        mat = quaternion_matrix(q)
        mat[:3, 3] = t
        return mat

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()