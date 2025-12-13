// point_cloud_simulator.cpp
#include "point_cloud_simulator.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/common/transforms.h>

#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>

std::string resolvePath(const std::string &path)
{
  std::string result = path;
  size_t find_pos = result.find("$(find ");
  if (find_pos != std::string::npos)
  {
    size_t end_pos = result.find(")", find_pos);
    if (end_pos != std::string::npos)
    {
      std::string package_name = result.substr(find_pos + 7, end_pos - find_pos - 7);
      std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
      std::string relative_path = result.substr(end_pos + 1);
      result = package_path + relative_path;
    }
  }
  return result;
}

// ==================================================================
// Constructor / Destructor
// ==================================================================
PointCloudSimulator::PointCloudSimulator()
{
  world_pc_.reset(new pcl::PointCloud<PointT>());
  kdtree_.reset(new pcl::KdTreeFLANN<PointT>());
}

PointCloudSimulator::~PointCloudSimulator() = default;

// ==================================================================
// 初始化
// ==================================================================
bool PointCloudSimulator::init(const std::string &pcd_path,
                               const FOVDescription &fov,
                               int pixel_per_deg,
                               bool downsample,
                               double lidar_downsample_voxel_size,
                               double points_noise_stderr)
{
  fov_ = fov;
  pixel_per_deg_ = pixel_per_deg;
  downsample_ = downsample;
  lidar_downsample_voxel_size_ = lidar_downsample_voxel_size;
  points_noise_stderr_ = points_noise_stderr;

  if (!load_pcd(pcd_path))
  {
    return false;
  }

  kdtree_->setInputCloud(world_pc_);

  // 预计算噪声池（滑动窗口）
  noise_window_size_ = std::max<size_t>(500, world_pc_->size() / 100);
  precompute_noise();

  RCLCPP_INFO(rclcpp::get_logger("PointCloudSimulator"),
              "PointCloudSimulator 初始化完成，地图点数: %zu", world_pc_->size());
  return true;
}

// ==================================================================
// 加载 PCD（支持 $(find pkg) 路径）
// ==================================================================
bool PointCloudSimulator::load_pcd(const std::string &pcd_path)
{
  std::string full_path = resolvePath(pcd_path);
  pcl::PointCloud<PointT> tmp;

  if (pcl::io::loadPCDFile<PointT>(full_path, tmp) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PointCloudSimulator"),
                 "无法加载 PCD 文件: %s", full_path.c_str());
    return false;
  }

  if (downsample_ && !tmp.empty())
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(lidar_downsample_voxel_size_,
                   lidar_downsample_voxel_size_,
                   lidar_downsample_voxel_size_);
    vg.setInputCloud(tmp.makeShared());
    vg.filter(*world_pc_);
    // (rclcpp::get_logger("PointCloudSimulator"),
    //             "加载并降采样 PCD: %s | 原始 %zu → 降采样后 %zu",
    //             fRCLCPP_INFOull_path.c_str(), tmp.size(), world_pc_->size());
  }
  else
  {
    *world_pc_ = std::move(tmp);
    // RCLCPP_INFO(rclcpp::get_logger("PointCloudSimulator"),
    //             "成功加载 PCD: %s | 点数: %zu", full_path.c_str(), world_pc_->size());
  }
  return true;
}

// ==================================================================
// 噪声预计算（滑动窗口）
// ==================================================================
void PointCloudSimulator::precompute_noise()
{
  noise_vec_.clear();
  std::default_random_engine rng(std::random_device{}());
  std::normal_distribution<double> nd(0.0, points_noise_stderr_);

  noise_vec_.reserve(noise_window_size_);
  for (size_t i = 0; i < noise_window_size_; ++i)
  {
    noise_vec_.push_back({nd(rng), nd(rng), nd(rng)});
  }
}

void PointCloudSimulator::update_noise_window()
{
  if (points_noise_stderr_ <= 0.0)
    return;

  std::default_random_engine rng(std::random_device{}());
  std::normal_distribution<double> nd(0.0, points_noise_stderr_);
  noise_vec_.erase(noise_vec_.begin());
  noise_vec_.push_back({nd(rng), nd(rng), nd(rng)});
}

// ==================================================================
// 主生成函数：世界坐标系网格采样 + 最近点保留（最快方案）
// ==================================================================
pcl::PointCloud<PointCloudSimulator::PointT>::Ptr
PointCloudSimulator::generate_scan(const Eigen::Isometry3d &world_to_lidar)
{
  using namespace std::chrono;

  // ------------------- 基础参数 -------------------
  const Eigen::Vector3d lidar_pos_w = world_to_lidar.inverse().translation();
  const double dist_min2 = fov_.dist_min * fov_.dist_min;
  const double dist_max2 = fov_.dist_max * fov_.dist_max;

  // ------------------- KdTree 搜索候选点 -------------------
  PointT search_pt{
    static_cast<float>(lidar_pos_w.x()), 
    static_cast<float>(lidar_pos_w.y()), 
    static_cast<float>(lidar_pos_w.z()), 
    0.f
  };
  std::vector<int> indices;
  std::vector<float> dist_sq;
  kdtree_->radiusSearch(search_pt, fov_.dist_max, indices, dist_sq);

  if (indices.empty())
  {
    return std::make_shared<pcl::PointCloud<PointT>>();
  }

  // ------------------- 角度网格设置（可控密度） -------------------
  const int yaw_bins = std::max(512, static_cast<int>((fov_.yaw_max_deg - fov_.yaw_min_deg) * pixel_per_deg_));
  const int pitch_bins = std::max(32, static_cast<int>((fov_.pitch_max_deg - fov_.pitch_min_deg) * pixel_per_deg_));

  const double yaw_min_rad = fov_.yaw_min_deg * M_PI / 180.0;
  const double yaw_span_rad = (fov_.yaw_max_deg - fov_.yaw_min_deg) * M_PI / 180.0;
  const double pitch_min_rad = fov_.pitch_min_deg * M_PI / 180.0;
  const double pitch_max_rad = fov_.pitch_max_deg * M_PI / 180.0;
  const double pitch_span_rad = (fov_.pitch_max_deg - fov_.pitch_min_deg) * M_PI / 180.0;

  const double sin_pitch_min = std::sin(pitch_min_rad);
  const double sin_pitch_max = std::sin(pitch_max_rad);

  // ------------------- 网格结构：只存最近点索引 -------------------
  struct GridCell
  {
    double dist = std::numeric_limits<double>::max();
    int idx = -1; // world_pc_ 中的索引
  };
  std::vector<GridCell> grid(yaw_bins * pitch_bins);

  // ------------------- 世界坐标系投影 + 最近点保留 -------------------
  for (size_t i = 0; i < indices.size(); ++i)
  {
    const int idx = indices[i];
    const PointT &p = (*world_pc_)[idx];

    const double dx = p.x - lidar_pos_w.x();
    const double dy = p.y - lidar_pos_w.y();
    const double dz = p.z - lidar_pos_w.z();

    const double d2 = dist_sq[i];
    if (d2 < dist_min2 || d2 > dist_max2)
      continue;

    const double dist = std::sqrt(d2);
    const double sin_pitch = dz / dist;
    if (sin_pitch < sin_pitch_min || sin_pitch > sin_pitch_max)
      continue;

    double yaw = std::atan2(dy, dx);
    double pitch = std::asin(sin_pitch);

    // Yaw 归一化到 [yaw_min, yaw_min + span)
    double norm_yaw = yaw - yaw_min_rad;
    while (norm_yaw < 0)
      norm_yaw += 2.0 * M_PI;
    while (norm_yaw >= yaw_span_rad)
      norm_yaw -= 2.0 * M_PI;
    if (norm_yaw < 0 || norm_yaw >= yaw_span_rad)
      continue;

    const int yi = static_cast<int>(norm_yaw / yaw_span_rad * yaw_bins);
    const int pi = static_cast<int>((pitch - pitch_min_rad) / pitch_span_rad * pitch_bins);

    if (yi < 0 || yi >= yaw_bins || pi < 0 || pi >= pitch_bins)
      continue;

    const int cell_idx = pi * yaw_bins + yi;
    if (dist < grid[cell_idx].dist)
    {
      grid[cell_idx].dist = dist;
      grid[cell_idx].idx = idx;
    }
  }

  // ------------------- 收集有效点索引 -------------------
  std::vector<int> valid_indices;
  valid_indices.reserve(grid.size() / 8); // 经验值
  for (const auto &cell : grid)
  {
    if (cell.idx != -1)
    {
      valid_indices.push_back(cell.idx);
    }
  }

  if (valid_indices.empty())
  {
    return std::make_shared<pcl::PointCloud<PointT>>();
  }

  // ------------------- 仅对少量点做坐标变换（关键提速！） -------------------
  auto cloud_final = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::transformPointCloud(*world_pc_, valid_indices, *cloud_final,
                           Eigen::Affine3d(world_to_lidar.matrix()));

  // ------------------- 加噪声 -------------------
  if (points_noise_stderr_ > 0.0 && !cloud_final->empty())
  {
    const size_t n = cloud_final->size();
    const size_t noiseN = noise_vec_.size();
    for (size_t i = 0; i < n; ++i)
    {
      const auto &nv = noise_vec_[i % noiseN];
      auto &p = cloud_final->points[i];
      p.x += nv.x();
      p.y += nv.y();
      p.z += nv.z();
    }
    update_noise_window();
  }
  return cloud_final;
}