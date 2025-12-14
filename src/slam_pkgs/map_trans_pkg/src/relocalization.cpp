#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <open3d/Open3D.h>
#include <open3d/io/PointCloudIO.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

class RelocalizationNode : public rclcpp::Node
{public:
    RelocalizationNode() : Node("relocalization_node")
    {
        // 参数声明
        this->declare_parameter("map_voxel_size", 0.3);
        this->declare_parameter("scan_voxel_size", 0.1);
        this->declare_parameter("localization_freq", 1.0);
        this->declare_parameter("localization_threshold", 0.8);
        this->declare_parameter("fov", 6.28318530718); // 2π
        this->declare_parameter("fov_far", 30.0);
        this->declare_parameter("tf_pub_freq", 50.0);
        this->declare_parameter("global_map_path", "");  // 新增：全局地图路径

        this->get_parameter("map_voxel_size", map_voxel_size_);
        this->get_parameter("scan_voxel_size", scan_voxel_size_);
        this->get_parameter("localization_freq", localization_freq_);
        this->get_parameter("localization_threshold", localization_threshold_);
        this->get_parameter("fov", fov_);
        this->get_parameter("fov_far", fov_far_);
        this->get_parameter("tf_pub_freq", tf_pub_freq_);

        std::string global_map_path;
        this->get_parameter("global_map_path", global_map_path);

        // 必要的发布者
        pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);
        pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);

        // 订阅者（移除 /map3d，保留 /initialpose）
        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&RelocalizationNode::initial_pose_callback, this, std::placeholders::_1));
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&RelocalizationNode::scan_callback, this, std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&RelocalizationNode::odom_callback, this, std::placeholders::_1));

        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 直接使用零位姿初始化
        T_map_to_odom_ = Eigen::Matrix4d::Identity();
        initial_pose_received_ = true;  // 视为已初始化

        // 加载全局地图
        if (global_map_path.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'global_map_path' is not set!");
            return;
        }

        if (!load_global_map(global_map_path))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to load global map from: %s", global_map_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Relocalization Node Initialized with zero initial pose.");
        RCLCPP_INFO(this->get_logger(), "Global map loaded from: %s", global_map_path.c_str());
    }

    ~RelocalizationNode()
    {
        if (localization_thread_.joinable())
        {
            localization_thread_.join();
        }
        if (tf_thread_.joinable())
        {
            tf_thread_.join();
        }
    }

    void start()
    {
        // 不再等待地图（已直接加载），也不再等待初始位姿（已设为零）
        // 只需等待第一次scan和odom即可进行首次定位（可选）

        RCLCPP_INFO(this->get_logger(), "Node starting with pre-loaded map and zero initial pose.");

        // 启动线程
        localization_thread_ = std::thread(&RelocalizationNode::localization_thread_func, this);
        tf_thread_ = std::thread(&RelocalizationNode::tf_thread_func, this);

        RCLCPP_INFO(this->get_logger(), "Relocalization threads started.");
    }

private:
    bool load_global_map(const std::string& pcd_path)
    {
        std::lock_guard<std::mutex> lock(global_map_mutex_);
        try
        {
            global_map_ = std::make_shared<open3d::geometry::PointCloud>();

            if (!open3d::io::ReadPointCloud(pcd_path, *global_map_))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read PCD file: %s", pcd_path.c_str());
                return false;
            }

            // 过滤无效点并体素降采样
            global_map_->RemoveNonFinitePoints();
            global_map_ = global_map_->VoxelDownSample(map_voxel_size_);

            global_map_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Global map loaded and downsampled. Points: %zu", global_map_->points_.size());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while loading map: %s", e.what());
            return false;
        }
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        try
        {
            tf2::Transform transform;
            tf2::fromMsg(msg->pose.pose, transform);

            T_map_to_odom_ = Eigen::Matrix4d::Identity();

            tf2::Matrix3x3 basis = transform.getBasis();
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << basis[0][0], basis[0][1], basis[0][2],
                               basis[1][0], basis[1][1], basis[1][2],
                               basis[2][0], basis[2][1], basis[2][2];
            T_map_to_odom_.block<3, 3>(0, 0) = rotation_matrix;

            tf2::Vector3 origin = transform.getOrigin();
            T_map_to_odom_.block<3, 1>(0, 3) = Eigen::Vector3d(origin.x(), origin.y(), origin.z());

            RCLCPP_INFO(this->get_logger(), "Initial pose updated manually via /initialpose.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing initial pose: %s", e.what());
        }
    }

    void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        try
        {
            // 转换ROS2点云到PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *pcl_cloud);

            // 转换PCL到Open3D
            cur_scan_ = std::make_shared<open3d::geometry::PointCloud>();
            cur_scan_->points_.reserve(pcl_cloud->size());
            for (const auto &point : pcl_cloud->points)
            {
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                {
                    cur_scan_->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                }
            }

            // 体素降采样
            cur_scan_ = cur_scan_->VoxelDownSample(scan_voxel_size_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing scan: %s", e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        cur_odom_ = msg;
        has_odom_ = true;
    }

    bool global_localization(Eigen::Matrix4d pose_estimation)
    {
        std::lock_guard<std::mutex> lock(relocalization_mutex_);
        if (!global_map_initialized_ || !cur_scan_ || !has_odom_)
        {
            RCLCPP_WARN(this->get_logger(), "Missing global map, scan, or odometry for localization");
            return false;
        }

        // RCLCPP_INFO(this->get_logger(), "Global localization by scan-to-map matching...");
        auto start_time = this->now();

        try
        {
            // 在FOV内裁剪全局地图
            auto global_map_in_FOV = crop_global_map_in_FOV(global_map_, pose_estimation, *cur_odom_);

            // 粗配准
            auto [transformation_coarse, fitness_coarse] = registration_at_scale(
                cur_scan_, global_map_in_FOV, pose_estimation, 5.0);

            // 精配准
            auto [transformation_fine, fitness_fine] = registration_at_scale(
                cur_scan_, global_map_in_FOV, transformation_coarse, 1.0);

            auto end_time = this->now();
            auto duration = end_time - start_time;

            // 当全局定位成功时才更新map2odom
            if (fitness_fine > localization_threshold_)
            {
                T_map_to_odom_ = transformation_fine;

                // 发布map_to_odom
                nav_msgs::msg::Odometry map_to_odom;
                map_to_odom.header.stamp = cur_odom_->header.stamp;
                map_to_odom.header.frame_id = "map";
                map_to_odom.child_frame_id = "odom";

                // 从变换矩阵提取位置和姿态
                Eigen::Vector3d translation = T_map_to_odom_.block<3, 1>(0, 3);
                Eigen::Matrix3d rotation = T_map_to_odom_.block<3, 3>(0, 0);
                Eigen::Quaterniond quat(rotation);

                map_to_odom.pose.pose.position.x = translation(0);
                map_to_odom.pose.pose.position.y = translation(1);
                map_to_odom.pose.pose.position.z = translation(2);
                map_to_odom.pose.pose.orientation.x = quat.x();
                map_to_odom.pose.pose.orientation.y = quat.y();
                map_to_odom.pose.pose.orientation.z = quat.z();
                map_to_odom.pose.pose.orientation.w = quat.w();

                pub_map_to_odom_->publish(map_to_odom);
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Not match! Fitness score: %.3f (threshold: %.3f)",
                            fitness_fine, localization_threshold_);
                return false;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during global localization: %s", e.what());
            return false;
        }
    }

    std::tuple<Eigen::Matrix4d, double> registration_at_scale(
        const std::shared_ptr<open3d::geometry::PointCloud> &source,
        const std::shared_ptr<open3d::geometry::PointCloud> &target,
        const Eigen::Matrix4d &initial,
        double scale)
    {
        // 设置ICP参数
        double voxel_size = scan_voxel_size_ * scale;
        double distance_threshold = 1.0 * scale;

        // 创建降采样点云
        auto source_down = source->VoxelDownSample(voxel_size);
        auto target_down = target->VoxelDownSample(voxel_size);

        using namespace open3d::pipelines::registration;

        // 创建收敛标准
        ICPConvergenceCriteria criteria;
        criteria.relative_fitness_ = 1e-6;
        criteria.relative_rmse_ = 1e-6;
        criteria.max_iteration_ = 20;

        // 执行ICP配准
        RegistrationResult result = RegistrationICP(
            *source_down, *target_down, distance_threshold,
            Eigen::Matrix4d(initial),
            TransformationEstimationPointToPoint(),
            criteria);

        return {result.transformation_, result.fitness_};
    }

    std::shared_ptr<open3d::geometry::PointCloud> crop_global_map_in_FOV(
        const std::shared_ptr<open3d::geometry::PointCloud> &global_map,
        const Eigen::Matrix4d &pose_estimation,
        const nav_msgs::msg::Odometry &cur_odom)
    {
        // 获取当前odom到base_link的变换
        tf2::Transform odom_to_base_link;
        tf2::fromMsg(cur_odom.pose.pose, odom_to_base_link);

        Eigen::Matrix4d T_odom_to_base_link = Eigen::Matrix4d::Identity();
        // 获取旋转矩阵
        tf2::Matrix3x3 basis = odom_to_base_link.getBasis();
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << basis[0][0], basis[0][1], basis[0][2],
            basis[1][0], basis[1][1], basis[1][2],
            basis[2][0], basis[2][1], basis[2][2];
        T_odom_to_base_link.block<3, 3>(0, 0) = rotation_matrix;

        // 获取平移向量
        tf2::Vector3 origin = odom_to_base_link.getOrigin();
        T_odom_to_base_link.block<3, 1>(0, 3) = Eigen::Vector3d(origin.x(), origin.y(), origin.z());

        // 计算map到base_link的变换
        Eigen::Matrix4d T_map_to_base_link = pose_estimation * T_odom_to_base_link;
        Eigen::Matrix4d T_base_link_to_map = T_map_to_base_link.inverse();

        // 将全局地图转换到base_link坐标系
        auto global_map_points = global_map->points_;
        std::vector<Eigen::Vector3d> points_in_base_link;
        points_in_base_link.reserve(global_map_points.size());

        for (const auto &point : global_map_points)
        {
            Eigen::Vector4d point_homogeneous(point(0), point(1), point(2), 1.0);
            Eigen::Vector4d transformed = T_base_link_to_map * point_homogeneous;
            points_in_base_link.push_back(transformed.head<3>());
        }

        // 根据FOV筛选点云
        std::vector<size_t> indices;
        for (size_t i = 0; i < points_in_base_link.size(); ++i)
        {
            const auto &p = points_in_base_link[i];
            double distance = p.norm();
            double angle = std::atan2(p(1), p(0));

            if (fov_ > M_PI)
            {
                // 环状LiDAR，仅过滤距离
                if (distance < fov_far_ && std::abs(angle) < fov_ / 2.0)
                {
                    indices.push_back(i);
                }
            }
            else
            {
                // 非环状LiDAR，保留前视范围
                if (p(0) > 0 && distance < fov_far_ && std::abs(angle) < fov_ / 2.0)
                {
                    indices.push_back(i);
                }
            }
        }

        // 创建FOV内的点云
        auto global_map_in_FOV = std::make_shared<open3d::geometry::PointCloud>();
        global_map_in_FOV->points_.reserve(indices.size());
        for (const auto &idx : indices)
        {
            global_map_in_FOV->points_.push_back(global_map_points[idx]);
        }

        return global_map_in_FOV;
    }

    void localization_thread_func()
    {
        rclcpp::Rate rate(localization_freq_);
        while (rclcpp::ok())
        {
            if (global_map_initialized_ && initial_pose_received_ && cur_scan_ && has_odom_)
            {
                global_localization(T_map_to_odom_);
            }
            rate.sleep();
        }
    }

    void tf_thread_func()
    {
        rclcpp::Rate rate(tf_pub_freq_);
        while (rclcpp::ok())
        {
            std::lock_guard<std::mutex> lock(tf_mutex_);
            if (has_odom_)
            {
                // 发布map到camera_init的TF
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = this->now();
                transform.header.frame_id = "map";
                transform.child_frame_id = "camera_init";

                // 从T_map_to_odom_提取变换
                Eigen::Vector3d translation = T_map_to_odom_.block<3, 1>(0, 3);
                Eigen::Matrix3d rotation = T_map_to_odom_.block<3, 3>(0, 0);
                Eigen::Quaterniond quat(rotation);

                transform.transform.translation.x = translation(0);
                transform.transform.translation.y = translation(1);
                transform.transform.translation.z = translation(2);
                transform.transform.rotation.x = quat.x();
                transform.transform.rotation.y = quat.y();
                transform.transform.rotation.z = quat.z();
                transform.transform.rotation.w = quat.w();

                tf_broadcaster_->sendTransform(transform);

                // 计算并发布/localization
                nav_msgs::msg::Odometry localization;
                localization.header.stamp = cur_odom_->header.stamp;
                localization.header.frame_id = "map";
                localization.child_frame_id = "body";

                // 获取odom到base_link的变换
                tf2::Transform odom_to_base_link;
                tf2::fromMsg(cur_odom_->pose.pose, odom_to_base_link);

                Eigen::Matrix4d T_odom_to_base_link = Eigen::Matrix4d::Identity();
                // 获取旋转矩阵
                tf2::Matrix3x3 basis = odom_to_base_link.getBasis();
                Eigen::Matrix3d rotation_matrix;
                rotation_matrix << basis[0][0], basis[0][1], basis[0][2],
                    basis[1][0], basis[1][1], basis[1][2],
                    basis[2][0], basis[2][1], basis[2][2];
                T_odom_to_base_link.block<3, 3>(0, 0) = rotation_matrix;

                // 获取平移向量
                tf2::Vector3 origin = odom_to_base_link.getOrigin();
                T_odom_to_base_link.block<3, 1>(0, 3) = Eigen::Vector3d(origin.x(), origin.y(), origin.z());

                // 计算map到base_link的变换
                Eigen::Matrix4d T_map_to_base_link = T_map_to_odom_ * T_odom_to_base_link;
                Eigen::Vector3d loc_translation = T_map_to_base_link.block<3, 1>(0, 3);
                Eigen::Matrix3d loc_rotation = T_map_to_base_link.block<3, 3>(0, 0);
                Eigen::Quaterniond loc_quat(loc_rotation);

                localization.pose.pose.position.x = loc_translation(0);
                localization.pose.pose.position.y = loc_translation(1);
                localization.pose.pose.position.z = loc_translation(2);
                localization.pose.pose.orientation.x = loc_quat.x();
                localization.pose.pose.orientation.y = loc_quat.y();
                localization.pose.pose.orientation.z = loc_quat.z();
                localization.pose.pose.orientation.w = loc_quat.w();

                // 复制速度信息
                localization.twist = cur_odom_->twist;
                pub_localization_->publish(localization);
            }
            rate.sleep();
        }
    }

    // 参数
    double map_voxel_size_;
    double scan_voxel_size_;
    double localization_freq_;
    double localization_threshold_;
    double fov_;
    double fov_far_;
    double tf_pub_freq_;

    // 点云数据
    std::shared_ptr<open3d::geometry::PointCloud> global_map_;
    std::shared_ptr<open3d::geometry::PointCloud> cur_scan_;
    bool global_map_initialized_ = false;

    // 位姿数据
    nav_msgs::msg::Odometry::SharedPtr cur_odom_;
    bool has_odom_ = false;
    Eigen::Matrix4d T_map_to_odom_;
    bool initial_pose_received_ = false;  // 现在启动时设为 true

    // 线程
    std::thread localization_thread_;
    std::thread tf_thread_;

    // 互斥锁
    std::mutex global_map_mutex_;
    std::mutex pose_mutex_;
    std::mutex scan_mutex_;
    std::mutex odom_mutex_;
    std::mutex relocalization_mutex_;
    std::mutex tf_mutex_;

    // ROS2接口
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizationNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}