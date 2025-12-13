#pragma once

#ifndef _POINT_CLOUD_SIMULATOR_HPP_
#define _POINT_CLOUD_SIMULATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct FOVDescription
{
  double dist_min, dist_max;
  double yaw_min_deg, yaw_max_deg;     // degrees
  double pitch_min_deg, pitch_max_deg; // degrees
};

class PointCloudSimulator
{
public:
  using PointT = pcl::PointXYZI;
  PointCloudSimulator();
  ~PointCloudSimulator();

  // 初始化：加载 pcd，设置参数
  bool init(const std::string &pcd_path,
            const FOVDescription &fov,
            int pixel_per_deg,
            bool downsample,
            double lidar_downsample_voxel_size,
            double points_noise_stderr);

  // 在给定 lidar 位姿（world -> lidar, as Eigen::Isometry3d）下生成一次扫描的点云（lidar_frame）
  pcl::PointCloud<PointT>::Ptr generate_scan(const Eigen::Isometry3d &world_to_lidar);

private:
  bool load_pcd(const std::string &pcd_path);
  void precompute_noise();
  void update_noise_window();

  pcl::PointCloud<PointT>::Ptr world_pc_;
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_;
  std::mutex mutex_;

  // params
  FOVDescription fov_;
  int pixel_per_deg_;
  bool downsample_;
  double lidar_downsample_voxel_size_;
  double points_noise_stderr_;
  // precomputed noise vectors (sliding window)
  std::vector<Eigen::Vector3d> noise_vec_;
  size_t noise_window_size_;

  // helper
  void world_to_lidar_cloud(const pcl::PointCloud<PointT>::Ptr &in,
                            pcl::PointCloud<PointT>::Ptr &out,
                            const Eigen::Isometry3d &world_to_lidar);
  void sphere_filter(pcl::PointCloud<PointT>::Ptr &pc);
  void downsample(pcl::PointCloud<PointT>::Ptr &pc);
  void apply_noise(pcl::PointCloud<PointT>::Ptr &pc);
  pcl::PointCloud<PointT>::Ptr occlusion_resample(pcl::PointCloud<PointT>::Ptr &pc);
};


#endif