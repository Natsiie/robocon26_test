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
{
public:
    RelocalizationNode() : Node("relocalization_node")
    {
        // 参数声明
        this->declare_parameter("map_voxel_size", 0.3);
        this->declare_parameter("scan_voxel_size", 0.1);
        this->declare_parameter("localization_freq", 1.0);
        // [修改] 降低默认阈值，避免实际上匹配还可以但被刷掉的情况
        this->declare_parameter("localization_threshold", 0.8); 
        this->declare_parameter("fov", 6.28318530718); // 2π
        this->declare_parameter("fov_far", 30.0);
        this->declare_parameter("tf_pub_freq", 50.0);
        this->declare_parameter("global_map_path", "");

        this->get_parameter("map_voxel_size", map_voxel_size_);
        this->get_parameter("scan_voxel_size", scan_voxel_size_);
        this->get_parameter("localization_freq", localization_freq_);
        this->get_parameter("localization_threshold", localization_threshold_);
        this->get_parameter("fov", fov_);
        this->get_parameter("fov_far", fov_far_);
        this->get_parameter("tf_pub_freq", tf_pub_freq_);

        std::string global_map_path;
        this->get_parameter("global_map_path", global_map_path);

        // 发布者
        pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);
        pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);

        // 订阅者
        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&RelocalizationNode::initial_pose_callback, this, std::placeholders::_1));
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&RelocalizationNode::scan_callback, this, std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&RelocalizationNode::odom_callback, this, std::placeholders::_1));

        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初始化位姿
        T_map_to_odom_ = Eigen::Matrix4d::Identity();
        initial_pose_received_ = true;

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

        RCLCPP_INFO(this->get_logger(), "Relocalization Node Initialized. Threshold: %.2f", localization_threshold_);
    }

    ~RelocalizationNode()
    {
        // 确保线程安全退出
        rclcpp::shutdown(); 
        if (localization_thread_.joinable()) localization_thread_.join();
        if (tf_thread_.joinable()) tf_thread_.join();
    }

    void start()
    {
        RCLCPP_INFO(this->get_logger(), "Node starting...");
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
            global_map_->RemoveNonFinitePoints();
            global_map_ = global_map_->VoxelDownSample(map_voxel_size_);
            global_map_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Global map loaded. Points: %zu", global_map_->points_.size());
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
        // [修改] 使用统一的 pose_mutex_
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *pcl_cloud);

            auto temp_scan = std::make_shared<open3d::geometry::PointCloud>();
            temp_scan->points_.reserve(pcl_cloud->size());
            for (const auto &point : pcl_cloud->points)
            {
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                {
                    temp_scan->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                }
            }
            // 更新成员变量
            cur_scan_ = temp_scan->VoxelDownSample(scan_voxel_size_);
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

    // [核心修改] 线程安全的全局定位函数
    bool global_localization(Eigen::Matrix4d pose_estimation_ignored)
    {
        // 1. 创建局部副本 (Local Copy)
        // 这样即使 callback 在后台更新了 cur_scan_，我们手中的 scan_ptr 依然指向有效的旧内存，不会段错误
        std::shared_ptr<open3d::geometry::PointCloud> scan_ptr;
        nav_msgs::msg::Odometry::SharedPtr odom_ptr;
        Eigen::Matrix4d current_T_map_odom;

        // 获取 Scan 锁
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (!cur_scan_) return false;
            scan_ptr = cur_scan_; // shared_ptr 拷贝，增加引用计数
        }

        // 获取 Odom 锁
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!has_odom_) return false;
            odom_ptr = cur_odom_;
        }
        
        // 获取当前估计的 T_map_to_odom (作为初值)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_T_map_odom = T_map_to_odom_;
        }

        // 检查全局地图
        if (!global_map_initialized_) return false;

        // 2. 执行耗时计算 (使用局部副本，无需持有锁)
        try
        {
            // 使用 odom_ptr 和 current_T_map_odom 计算预测位姿
            // 这里我们不需要传入 pose_estimation 参数，直接用内部维护的 T_map_to_odom_
            auto global_map_in_FOV = crop_global_map_in_FOV(global_map_, current_T_map_odom, *odom_ptr);

            // 粗配准
            auto [transformation_coarse, fitness_coarse] = registration_at_scale(
                scan_ptr, global_map_in_FOV, current_T_map_odom, 5.0);

            // 精配准
            auto [transformation_fine, fitness_fine] = registration_at_scale(
                scan_ptr, global_map_in_FOV, transformation_coarse, 1.0);

            // 3. 更新结果
            if (fitness_fine > localization_threshold_)
            {
                // 获取锁更新位姿
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    T_map_to_odom_ = transformation_fine;
                }

                // 日志
                // RCLCPP_INFO(this->get_logger(), "Loc Success! Fitness: %.3f", fitness_fine);
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Not match! Fitness: %.3f (Thresh: %.3f)",
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
        double voxel_size = scan_voxel_size_ * scale;
        double distance_threshold = 1.0 * scale;

        // VoxelDownSample 会返回新的指针，线程安全
        auto source_down = source->VoxelDownSample(voxel_size);
        auto target_down = target->VoxelDownSample(voxel_size);

        using namespace open3d::pipelines::registration;

        ICPConvergenceCriteria criteria;
        criteria.relative_fitness_ = 1e-6;
        criteria.relative_rmse_ = 1e-6;
        criteria.max_iteration_ = 20;

        RegistrationResult result = RegistrationICP(
            *source_down, *target_down, distance_threshold,
            Eigen::Matrix4d(initial),
            TransformationEstimationPointToPoint(),
            criteria);

        return {result.transformation_, result.fitness_};
    }

    std::shared_ptr<open3d::geometry::PointCloud> crop_global_map_in_FOV(
        const std::shared_ptr<open3d::geometry::PointCloud> &global_map,
        const Eigen::Matrix4d &map_to_odom_mat,
        const nav_msgs::msg::Odometry &cur_odom)
    {
        // 1. 获取 Odom -> Base
        tf2::Transform odom_to_base_tf;
        tf2::fromMsg(cur_odom.pose.pose, odom_to_base_tf);
        
        // 转换为 Eigen
        Eigen::Matrix4d T_odom_to_base = Eigen::Matrix4d::Identity();
        tf2::Matrix3x3 basis = odom_to_base_tf.getBasis();
        for(int r=0; r<3; r++)
            for(int c=0; c<3; c++) 
                T_odom_to_base(r,c) = basis[r][c];
        
        tf2::Vector3 origin = odom_to_base_tf.getOrigin();
        T_odom_to_base.block<3,1>(0,3) = Eigen::Vector3d(origin.x(), origin.y(), origin.z());

        // 2. 计算 Map -> Base
        Eigen::Matrix4d T_map_to_base = map_to_odom_mat * T_odom_to_base;
        Eigen::Matrix4d T_base_to_map = T_map_to_base.inverse();

        // 3. 转换点云 (注意：这里如果地图非常大会比较慢，但 Open3D 操作通常较快)
        // 这里的 global_map 是只读的，线程安全
        auto global_map_points = global_map->points_;
        std::vector<size_t> indices;
        indices.reserve(global_map_points.size());

        for (size_t i = 0; i < global_map_points.size(); ++i)
        {
            Eigen::Vector3d p_map = global_map_points[i];
            Eigen::Vector4d p_base = T_base_to_map * Eigen::Vector4d(p_map(0), p_map(1), p_map(2), 1.0);
            
            double x = p_base(0);
            double y = p_base(1);
            double distance = p_base.head<3>().norm();
            double angle = std::atan2(y, x);

            if (distance < fov_far_)
            {
                 // 简单的FOV判断
                 if (fov_ > M_PI || (x > 0 && std::abs(angle) < fov_ / 2.0)) {
                     indices.push_back(i);
                 }
            }
        }

        return global_map->SelectByIndex(indices);
    }

    void localization_thread_func()
    {
        rclcpp::Rate rate(localization_freq_);
        while (rclcpp::ok())
        {
            // 在这里不加锁，锁的逻辑下沉到 global_localization 内部
            // 只需要判断标志位（bool读写是原子的，或者不影响逻辑）
            if (global_map_initialized_ && initial_pose_received_)
            {
                // 参数实际上在函数内部重新获取了，这里传 Identity 或者类成员都行
                // 为了兼容接口，传个空矩阵也没事，因为内部用了 current_T_map_odom
                global_localization(Eigen::Matrix4d::Identity());
            }
            rate.sleep();
        }
    }

    void tf_thread_func()
    {
        rclcpp::Rate rate(tf_pub_freq_);
        while (rclcpp::ok())
        {
            // [核心修改] 使用局部副本，避免长时间持有锁
            Eigen::Matrix4d T_map_odom_copy;
            nav_msgs::msg::Odometry::SharedPtr odom_copy;
            bool ready = false;

            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                T_map_odom_copy = T_map_to_odom_;
            }

            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                if(has_odom_) {
                    odom_copy = cur_odom_;
                    ready = true;
                }
            }

            if (ready)
            {
                // 发布 map -> camera_init (odom frame)
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = this->now();
                transform.header.frame_id = "map";
                transform.child_frame_id = "camera_init"; // 或者是 odom，看你的TF树定义

                Eigen::Vector3d t = T_map_odom_copy.block<3, 1>(0, 3);
                Eigen::Matrix3d r = T_map_odom_copy.block<3, 3>(0, 0);
                Eigen::Quaterniond q(r);

                transform.transform.translation.x = t(0);
                transform.transform.translation.y = t(1);
                transform.transform.translation.z = t(2);
                transform.transform.rotation.x = q.x();
                transform.transform.rotation.y = q.y();
                transform.transform.rotation.z = q.z();
                transform.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(transform);

                // 发布 /localization 话题
                // 计算 Map -> Body
                nav_msgs::msg::Odometry localization;
                localization.header.stamp = odom_copy->header.stamp;
                localization.header.frame_id = "map";
                localization.child_frame_id = "body";

                tf2::Transform odom_to_body;
                tf2::fromMsg(odom_copy->pose.pose, odom_to_body);
                
                // 简单的 TF 乘法: T_map_body = T_map_odom * T_odom_body
                // 手动实现简单的矩阵乘法
                Eigen::Matrix4d T_odom_body = Eigen::Matrix4d::Identity();
                tf2::Matrix3x3 basis = odom_to_body.getBasis();
                for(int i=0;i<3;i++) for(int j=0;j<3;j++) T_odom_body(i,j) = basis[i][j];
                tf2::Vector3 origin = odom_to_body.getOrigin();
                T_odom_body(0,3) = origin.x(); T_odom_body(1,3) = origin.y(); T_odom_body(2,3) = origin.z();

                Eigen::Matrix4d T_map_body = T_map_odom_copy * T_odom_body;
                
                Eigen::Vector3d loc_t = T_map_body.block<3,1>(0,3);
                Eigen::Quaterniond loc_q(T_map_body.block<3,3>(0,0));

                localization.pose.pose.position.x = loc_t(0);
                localization.pose.pose.position.y = loc_t(1);
                localization.pose.pose.position.z = loc_t(2);
                localization.pose.pose.orientation.x = loc_q.x();
                localization.pose.pose.orientation.y = loc_q.y();
                localization.pose.pose.orientation.z = loc_q.z();
                localization.pose.pose.orientation.w = loc_q.w();

                localization.twist = odom_copy->twist;
                pub_localization_->publish(localization);
            }
            rate.sleep();
        }
    }

    // 变量
    double map_voxel_size_, scan_voxel_size_;
    double localization_freq_, localization_threshold_;
    double fov_, fov_far_, tf_pub_freq_;

    std::shared_ptr<open3d::geometry::PointCloud> global_map_;
    std::shared_ptr<open3d::geometry::PointCloud> cur_scan_;
    bool global_map_initialized_ = false;

    nav_msgs::msg::Odometry::SharedPtr cur_odom_;
    bool has_odom_ = false;
    
    Eigen::Matrix4d T_map_to_odom_;
    bool initial_pose_received_ = false;

    std::thread localization_thread_;
    std::thread tf_thread_;

    // [修改] 锁定义：只保留数据锁
    std::mutex global_map_mutex_; // 保护 global_map_
    std::mutex pose_mutex_;       // 保护 T_map_to_odom_
    std::mutex scan_mutex_;       // 保护 cur_scan_
    std::mutex odom_mutex_;       // 保护 cur_odom_

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
