#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "point_cloud_simulator.hpp"
#include "imu_data_generate.hpp"

class VirtualLidarNode : public rclcpp::Node
{
public:
    VirtualLidarNode() : Node("virtual_lidar_driver_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_)
    {
        // 创建独立的回调组用于IMU定时器（可选，高频）
        imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // params
        this->declare_parameter<std::string>("world_frame_id", "world");
        this->declare_parameter<std::string>("lidar_frame_id", "lidar_frame");
        this->declare_parameter<std::string>("imu_topic", "/sim/imu");
        this->declare_parameter<std::string>("lidar_topic", "/sim/points");
        this->declare_parameter<double>("imu_update_rate", 200.0); // Mid-360: 200Hz
        this->declare_parameter<double>("lidar_update_rate", 10.0);
        this->declare_parameter<std::string>("pcd_path", "map.pcd");
        this->declare_parameter<int>("pixel_per_deg", 1);
        this->declare_parameter<bool>("downsample", true);
        this->declare_parameter<double>("lidar_downsample_voxel_size", 0.05);
        this->declare_parameter<double>("points_noise_stderr", 0.01);
        this->declare_parameter<double>("fov.dist_min", 0.5);
        this->declare_parameter<double>("fov.dist_max", 50.0);
        this->declare_parameter<double>("fov.yaw_min_deg", -180.0);
        this->declare_parameter<double>("fov.yaw_max_deg", 180.0);
        this->declare_parameter<double>("fov.pitch_min_deg", -30.0);
        this->declare_parameter<double>("fov.pitch_max_deg", 10.0);
        this->declare_parameter<bool>("imu.with_3dof_model", false);
        this->declare_parameter<double>("imu.acc_cov", 0.0005); // Mid-360-like
        this->declare_parameter<double>("imu.gyro_cov", 1e-5);  // Mid-360-like
        this->declare_parameter<std::vector<double>>("imu.gravity_vector", std::vector<double>{0.0, 0.0, -9.81});

        world_frame_id_ = this->get_parameter("world_frame_id").as_string();
        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        lidar_topic_ = this->get_parameter("lidar_topic").as_string();
        imu_rate_ = this->get_parameter("imu_update_rate").as_double();
        lidar_rate_ = this->get_parameter("lidar_update_rate").as_double();

        // fov
        FOVDescription fov;
        fov.dist_min = this->get_parameter("fov.dist_min").as_double();
        fov.dist_max = this->get_parameter("fov.dist_max").as_double();
        fov.yaw_min_deg = this->get_parameter("fov.yaw_min_deg").as_double();
        fov.yaw_max_deg = this->get_parameter("fov.yaw_max_deg").as_double();
        fov.pitch_min_deg = this->get_parameter("fov.pitch_min_deg").as_double();
        fov.pitch_max_deg = this->get_parameter("fov.pitch_max_deg").as_double();

        // other params
        std::string pcd_path = this->get_parameter("pcd_path").as_string();
        int pixel_per_deg = this->get_parameter("pixel_per_deg").as_int();
        bool downsample = this->get_parameter("downsample").as_bool();
        double voxel_size = this->get_parameter("lidar_downsample_voxel_size").as_double();
        double points_noise_stderr = this->get_parameter("points_noise_stderr").as_double();

        // imu params: Mid-360-like
        bool imu_3dof = this->get_parameter("imu.with_3dof_model").as_bool();
        double acc_cov = this->get_parameter("imu.acc_cov").as_double();
        double gyro_cov = this->get_parameter("imu.gyro_cov").as_double();
        auto gv = this->get_parameter("imu.gravity_vector").as_double_array();
        Eigen::Vector3d gravity(gv[0], gv[1], gv[2]);

        // publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 100); // 更大队列 for 高频
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_, 10);

        // setup simulators
        pcsim_.reset(new PointCloudSimulator());
        if (!pcsim_->init(pcd_path, fov, pixel_per_deg, downsample, voxel_size, points_noise_stderr))
        {
            RCLCPP_ERROR(this->get_logger(), "PointCloudSimulator init failed");
        }

        imugen_.reset(new ImuDataGenerator());
        imugen_->init(imu_3dof, acc_cov, gyro_cov, gravity);

        // timers: IMU高频，但为测试FAST-LIO，优先在lidar_cb中发布同步IMU
        imu_timer_ = this->create_wall_timer(  // 注释：若需独立200Hz，取消注释
            std::chrono::duration<double>(1.0 / imu_rate_),
            std::bind(&VirtualLidarNode::imu_timer_cb, this),
            imu_callback_group_
        );

        lidar_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / lidar_rate_),
            std::bind(&VirtualLidarNode::lidar_timer_cb, this),
            lidar_callback_group_);

        RCLCPP_INFO(this->get_logger(), "VirtualLidarNode started with Mid-360-like IMU (200Hz capable, low-noise)");
    }

private:
    // params
    std::string world_frame_id_, lidar_frame_id_, imu_topic_, lidar_topic_;
    double imu_rate_, lidar_rate_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    std::unique_ptr<PointCloudSimulator> pcsim_;
    std::unique_ptr<ImuDataGenerator> imugen_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;

    void imu_timer_cb() // 可选高频发布
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_.lookupTransform(world_frame_id_, lidar_frame_id_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            return;
        }
        Eigen::Isometry3d world_to_lidar = tf2::transformToEigen(t);
        rclcpp::Time stamp = this->now(); // 当前时间 for 高频
        auto imu = imugen_->update_imu(world_to_lidar, stamp);
        imu.header.frame_id = lidar_frame_id_;
        imu_pub_->publish(imu);
    }

    void lidar_timer_cb()
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_.lookupTransform(world_frame_id_, lidar_frame_id_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            return;
        }
        Eigen::Isometry3d world_to_lidar = tf2::transformToEigen(t).inverse(); // 修正：lidar生成需world_to_lidar
        rclcpp::Time stamp = t.header.stamp;

        // generate pointcloud in lidar frame
        auto pc = pcsim_->generate_scan(world_to_lidar);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = lidar_frame_id_;
        pc_pub_->publish(msg);

        // 同步IMU发布（10Hz，适合FAST-LIO测试；高频见 imu_timer_cb ）
        auto imu = imugen_->update_imu(world_to_lidar, stamp);
        imu.header.frame_id = lidar_frame_id_;
        imu_pub_->publish(imu);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VirtualLidarNode>();

    // 多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}