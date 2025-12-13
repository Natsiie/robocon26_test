// src/imu_data_generate.cpp
#include "imu_data_generate.hpp"
#include <iostream> // for debug

ImuDataGenerator::ImuDataGenerator()
    : rng_(std::random_device{}()),
      noise_gyro_(0.0, 1.0),
      noise_acc_(0.0, 1.0)
{
}

ImuDataGenerator::~ImuDataGenerator() = default;

void ImuDataGenerator::init(bool with_3dof_model,
                            double acc_cov,
                            double gyro_cov,
                            const Eigen::Vector3d &gravity_vector,
                            bool use_real_motion_acc,
                            double motion_noise_scale)
{
  with_3dof_ = with_3dof_model;
  acc_cov_ = acc_cov;
  gyro_cov_ = gyro_cov;
  gravity_ = gravity_vector;
  use_real_motion_acc_ = use_real_motion_acc;
  motion_noise_scale_ = motion_noise_scale;

  initialized_ = false;
  yaw_initialized_ = false;
  gyro_bias_.setZero();
  acc_bias_.setZero();
}

// computeAngularVelocity（不变）
Eigen::Vector3d ImuDataGenerator::computeAngularVelocity(
    const Eigen::Quaterniond &q_curr,
    const Eigen::Quaterniond &q_prev,
    double dt) const
{
  if (dt < 1e-6)
    return Eigen::Vector3d::Zero();

  Eigen::Quaterniond dq = q_curr * q_prev.inverse();
  if (dq.w() < 0.0)
    dq.coeffs() = -dq.coeffs();

  double angle = 2.0 * std::acos(std::clamp(dq.w(), -1.0, 1.0));

  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  if (angle > 1e-8)
  {
    double s = std::sin(angle / 2.0);
    if (std::abs(s) > 1e-8)
      axis = dq.vec() / s;
  }

  if (angle > M_PI)
    angle -= 2.0 * M_PI;

  return (angle / dt) * axis;
}

sensor_msgs::msg::Imu ImuDataGenerator::update_imu(
    const Eigen::Isometry3d &T_world_to_lidar,
    const rclcpp::Time &stamp)
{
  Eigen::Vector3d pos = T_world_to_lidar.translation();
  Eigen::Matrix3d R = T_world_to_lidar.rotation();
  Eigen::Quaterniond q(R);
  q = normalizeQ(q);

  // 3DoF 处理（不变）
  if (with_3dof_)
  {
    double yaw_raw = std::atan2(R(1, 0), R(0, 0));

    if (!yaw_initialized_)
    {
      prev_yaw_raw_ = yaw_raw;
      prev_yaw_continuous_ = yaw_raw;
      yaw_initialized_ = true;
    }

    double dyaw = shortest_angular_distance(prev_yaw_raw_, yaw_raw);
    prev_yaw_continuous_ += dyaw;
    prev_yaw_raw_ = yaw_raw;

    Eigen::AngleAxisd aa(prev_yaw_continuous_, Eigen::Vector3d::UnitZ());
    q = Eigen::Quaterniond(aa).normalized();
    pos.z() = 0.0;
  }

  if (!initialized_)
  {
    prev_q_ = q;
    prev_pos_ = pos;
    prev_time_ = stamp;
    prev_lin_vel_.setZero();
    initialized_ = true;

    Eigen::Vector3d acc_body = -q.toRotationMatrix().transpose() * gravity_;
    return build_msg(q, Eigen::Vector3d::Zero(), acc_body, stamp);
  }

  double dt = (stamp - prev_time_).seconds();
  if (dt <= 0.0)
    dt = 0.001;

  // 角速度（不变）
  Eigen::Vector3d gyro;
  if (with_3dof_)
  {
    double yaw_raw = std::atan2(R(1, 0), R(0, 0));
    double dyaw = shortest_angular_distance(prev_yaw_raw_, yaw_raw);
    prev_yaw_raw_ = yaw_raw;
    gyro = Eigen::Vector3d(0.0, 0.0, dyaw / dt);
  }
  else
  {
    gyro = computeAngularVelocity(q, prev_q_, dt);
  }

  // 加速度：混合模式 - 重力 + 真实运动（关键修正）
  Eigen::Vector3d acc_world = gravity_; // 默认重力
  if (use_real_motion_acc_)
  {
    // 数值微分：速度 & 加速度（world frame）
    Eigen::Vector3d vel = (pos - prev_pos_) / dt;
    Eigen::Vector3d acc_motion = (vel - prev_lin_vel_) / dt; // 线性acc
    acc_world += acc_motion;                                 // 叠加运动acc（模拟高速）

    // 添加运动噪声（低值，确保平滑但有激励）
    acc_motion += Eigen::Vector3d(noise_acc_(rng_), noise_acc_(rng_), noise_acc_(rng_)) * motion_noise_scale_;
  }

  Eigen::Vector3d acc_body = q.toRotationMatrix().transpose() * acc_world; // 转body frame

  if (with_3dof_)
    acc_body.z() = 0.0;

  // 噪声 + 偏置（Mid-360低值）
  double sigma_g = std::sqrt(gyro_cov_ / dt);
  double sigma_a = std::sqrt(acc_cov_ / dt);

  gyro += gyro_bias_ + Eigen::Vector3d(noise_gyro_(rng_), noise_gyro_(rng_), noise_gyro_(rng_)) * sigma_g;
  acc_body += acc_bias_ + Eigen::Vector3d(noise_acc_(rng_), noise_acc_(rng_), noise_acc_(rng_)) * sigma_a;

  // 偏置游走：极低（Mid-360稳定性）
  double bias_walk_rate = 1e-10; // 进一步降低
  gyro_bias_ += Eigen::Vector3d(noise_gyro_(rng_), noise_gyro_(rng_), noise_gyro_(rng_)) * std::sqrt(gyro_cov_ * bias_walk_rate);
  acc_bias_ += Eigen::Vector3d(noise_acc_(rng_), noise_acc_(rng_), noise_acc_(rng_)) * std::sqrt(acc_cov_ * bias_walk_rate);

  prev_q_ = q;
  prev_pos_ = pos;
  prev_lin_vel_ = (pos - prev_pos_) / dt;
  prev_time_ = stamp;

  return build_msg(q, gyro, acc_body, stamp);
}

// build_msg（不变，协方差更新为Mid-360值）
sensor_msgs::msg::Imu ImuDataGenerator::build_msg(
    const Eigen::Quaterniond &q,
    const Eigen::Vector3d &gyro,
    const Eigen::Vector3d &acc,
    const rclcpp::Time &stamp) const
{
  sensor_msgs::msg::Imu msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = "lidar_frame";

  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  msg.angular_velocity.x = gyro.x();
  msg.angular_velocity.y = gyro.y();
  msg.angular_velocity.z = gyro.z();

  msg.linear_acceleration.x = acc.x();
  msg.linear_acceleration.y = acc.y();
  msg.linear_acceleration.z = acc.z();

  std::fill(msg.orientation_covariance.begin(), msg.orientation_covariance.end(), 0.0);
  msg.orientation_covariance[0] = -1;

  // Mid-360协方差
  msg.angular_velocity_covariance = {gyro_cov_, 0, 0, 0, gyro_cov_, 0, 0, 0, gyro_cov_};
  msg.linear_acceleration_covariance = {acc_cov_, 0, 0, 0, acc_cov_, 0, 0, 0, acc_cov_};

  return msg;
}