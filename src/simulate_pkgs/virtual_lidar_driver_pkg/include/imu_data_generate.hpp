#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

class ImuDataGenerator
{
public:
  ImuDataGenerator();
  ~ImuDataGenerator();

  void init(bool with_3dof_model,
            double acc_cov,
            double gyro_cov,
            const Eigen::Vector3d &gravity_vector = Eigen::Vector3d(0, 0, -9.81),
            bool use_real_motion_acc = true,  // 新参数：启用真实运动acc
            double motion_noise_scale = 0.1); // 运动acc额外噪声缩放（低值=平滑）

  sensor_msgs::msg::Imu update_imu(const Eigen::Isometry3d &T_world_to_lidar,
                                   const rclcpp::Time &stamp);

private:
  // 参数
  bool with_3dof_{false};
  double acc_cov_{1e-5};  // Mid-360: ~100 μg/√Hz → 协方差~1e-5 m²/s⁴
  double gyro_cov_{1e-6}; // Mid-360: ~0.007°/s/√Hz → ~1e-6 rad²/s²
  Eigen::Vector3d gravity_{0, 0, -9.81};
  bool use_real_motion_acc_{true}; // 新增
  double motion_noise_scale_{0.1}; // 新增

  // 状态（不变）
  bool initialized_{false};
  bool yaw_initialized_{false};
  double prev_yaw_raw_{0.0};
  double prev_yaw_continuous_{0.0};

  Eigen::Quaterniond prev_q_;
  Eigen::Vector3d prev_pos_;
  Eigen::Vector3d prev_lin_vel_;
  rclcpp::Time prev_time_;

  // 偏置 & 噪声
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_bias_{Eigen::Vector3d::Zero()};

  std::mt19937 rng_;
  std::normal_distribution<double> noise_gyro_;
  std::normal_distribution<double> noise_acc_;

  // 工具函数（不变）
  Eigen::Quaterniond normalizeQ(const Eigen::Quaterniond &q) const
  {
    return Eigen::Quaterniond(q.coeffs().normalized());
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  static double shortest_angular_distance(double from, double to)
  {
    double diff = normalize_angle(to - from);
    if (diff > M_PI)
      diff -= 2.0 * M_PI;
    else if (diff < -M_PI)
      diff += 2.0 * M_PI;
    return diff;
  }

  Eigen::Vector3d computeAngularVelocity(const Eigen::Quaterniond &q_curr,
                                         const Eigen::Quaterniond &q_prev,
                                         double dt) const;

  sensor_msgs::msg::Imu build_msg(const Eigen::Quaterniond &q,
                                  const Eigen::Vector3d &gyro,
                                  const Eigen::Vector3d &acc,
                                  const rclcpp::Time &stamp) const;
};