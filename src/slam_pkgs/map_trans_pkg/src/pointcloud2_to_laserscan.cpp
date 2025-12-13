// src/map_trans_pkg/src/pointcloud2_to_laserscan_for_body.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <memory>
#include <vector>
#include <cmath>

class PointCloudToLaserScanForBody : public rclcpp::Node
{
public:
    PointCloudToLaserScanForBody() : Node("pointcloud_to_laserscan_for_body")
    {
        // 参数
        declare_parameter("pc_topic", "/livox/lidar_pc2");
        declare_parameter("laserscan_topic", "/scan");
        declare_parameter("lidar_frame", "robot_lidar");
        declare_parameter("base_frame", "base_link");
        declare_parameter("output_frame", "body");
        declare_parameter("min_height", -0.2);
        declare_parameter("max_height", 0.2);
        declare_parameter("min_range", 0.1);
        declare_parameter("max_range", 10.0);
        declare_parameter("angle_min_deg", -180.0);
        declare_parameter("angle_max_deg", 180.0);
        declare_parameter("angle_resolution_deg", 0.5);
        declare_parameter("scan_time", 0.1);

        // 获取参数
        pc_topic_ = get_parameter("pc_topic").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        output_frame_ = get_parameter("output_frame").as_string();

        min_height_ = get_parameter("min_height").as_double();
        max_height_ = get_parameter("max_height").as_double();
        min_range_ = get_parameter("min_range").as_double();
        max_range_ = get_parameter("max_range").as_double();
        scan_time_ = get_parameter("scan_time").as_double();

        double min_deg = get_parameter("angle_min_deg").as_double();
        double max_deg = get_parameter("angle_max_deg").as_double();
        double res_deg = get_parameter("angle_resolution_deg").as_double();

        angle_min_ = min_deg * M_PI / 180.0;
        angle_max_ = max_deg * M_PI / 180.0;
        angle_inc_ = res_deg * M_PI / 180.0;
        num_readings_ = static_cast<int>(std::round((angle_max_ - angle_min_) / angle_inc_)) + 1;

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅发布
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic_, 10,
            std::bind(&PointCloudToLaserScanForBody::callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        RCLCPP_INFO(this->get_logger(),
                    "Converting pointcloud from [%s] → project in [%s] → publish as frame_id=[%s]",
                    lidar_frame_.c_str(), base_frame_.c_str(), output_frame_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id != lidar_frame_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                                 "Expected frame %s, got %s", lidar_frame_.c_str(), msg->header.frame_id.c_str());
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                base_frame_,          // target
                msg->header.frame_id, // source (通常是 robot_lidar)
                msg->header.stamp,    // 用点云时间戳更准
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                                 "TF lookup failed (%s → %s): %s",
                                 msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        // 变换点云到 base_link 坐标系
        sensor_msgs::msg::PointCloud2 cloud_in_base;
        tf2::doTransform(*msg, cloud_in_base, transform);

        // 转为 PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(cloud_in_base, cloud);

        // 高度过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*filtered);

        // 投影到极坐标，取最近点
        std::vector<float> ranges(num_readings_, std::numeric_limits<float>::infinity());

        for (const auto &p : filtered->points)
        {
            float range = std::hypot(p.x, p.y);
            if (range < min_range_ || range > max_range_)
                continue;

            float angle = std::atan2(p.y, p.x);
            if (angle < angle_min_)
                angle += 2 * M_PI;
            if (angle > angle_max_)
                angle -= 2 * M_PI;
            if (angle < angle_min_ || angle > angle_max_)
                continue;

            int idx = static_cast<int>((angle - angle_min_) / angle_inc_);
            if (idx >= 0 && idx < num_readings_ && range < ranges[idx])
            {
                ranges[idx] = range;
            }
        }

        // 发布：frame_id 强制为 body
        auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = transform.header.stamp;
        scan->header.frame_id = output_frame_;
        scan->angle_min = angle_min_;
        scan->angle_max = angle_max_;
        scan->angle_increment = angle_inc_;
        scan->scan_time = scan_time_;
        scan->range_min = min_range_;
        scan->range_max = max_range_;
        scan->ranges = ranges;
        scan->intensities.assign(num_readings_, 0.0f);

        pub_->publish(std::move(scan));
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    std::string pc_topic_, lidar_frame_, base_frame_, output_frame_;
    double min_height_, max_height_, min_range_, max_range_, scan_time_;
    double angle_min_, angle_max_, angle_inc_;
    int num_readings_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserScanForBody>());
    rclcpp::shutdown();
    return 0;
}