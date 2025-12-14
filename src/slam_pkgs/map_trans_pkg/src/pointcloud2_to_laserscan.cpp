// src/map_trans_pkg/src/pointcloud2_to_laserscan_for_body.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
        declare_parameter("lidar_frame", "body");           // lidar frame (usually fixed to body)
        declare_parameter("base_frame", "body");            // body frame
        declare_parameter("stable_frame", "map");           // or "odom" – the frame assumed horizontal
        declare_parameter("output_frame", "body_2d");
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
        laserscan_topic_ = get_parameter("laserscan_topic").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        stable_frame_ = get_parameter("stable_frame").as_string();
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
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅点云
        pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic_, 10,
            std::bind(&PointCloudToLaserScanForBody::pc_callback, this, std::placeholders::_1));

        // 发布 LaserScan
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic_, 10);

        // 定时发布 body → body_2d 变换（10 Hz）
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PointCloudToLaserScanForBody::publish_level_transform, this));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Using %s → %s to level the frame %s",
                    stable_frame_.c_str(), base_frame_.c_str(), output_frame_.c_str());
    }

private:
    void publish_level_transform()
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            // Use latest available transform 
            tf_stamped = tf_buffer_->lookupTransform(stable_frame_, base_frame_ , rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            return;  // No transform yet, skip broadcasting
        }

        // Extract full RPY from stable_frame → body (map/odom → body)
        tf2::Quaternion q;
        tf2::fromMsg(tf_stamped.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Create leveling rotation: cancel roll and pitch (align Z with gravity)
        tf2::Quaternion q_level;
        q_level.setRPY(-roll, -pitch, 0.0);

        // Then apply the projected yaw around the new vertical Z
        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);

        tf2::Quaternion q_final = q_yaw * q_level;  // Level first, then yaw

        // Broadcast body → body_2d
        geometry_msgs::msg::TransformStamped level_tf;
        level_tf.header.stamp = this->now();
        level_tf.header.frame_id = base_frame_;      // body
        level_tf.child_frame_id = output_frame_;     // body_2d
        level_tf.transform.translation.x = 0.0;
        level_tf.transform.translation.y = 0.0;
        level_tf.transform.translation.z = -tf_stamped.transform.translation.z;
        level_tf.transform.rotation = tf2::toMsg(q_final);

        tf_broadcaster_->sendTransform(level_tf);
    }
    // 点云回调
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id != lidar_frame_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                                 "Expected pointcloud frame %s, got %s",
                                 lidar_frame_.c_str(), msg->header.frame_id.c_str());
        }

        // Step 1: lidar → body (通常是固定变换)
        geometry_msgs::msg::TransformStamped tf_lidar_to_body;
        try {
            tf_lidar_to_body = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "TF lidar→body failed: %s", ex.what());
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_body;
        tf2::doTransform(*msg, cloud_body, tf_lidar_to_body);

        // Step 2: body → body_2d
        geometry_msgs::msg::TransformStamped tf_body_to_2d;
        try {
            tf_body_to_2d = tf_buffer_->lookupTransform(output_frame_, base_frame_,rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                                 "TF body -> body_2d not available yet: %s", ex.what());
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_2d;
        tf2::doTransform(cloud_body, cloud_2d, tf_body_to_2d);

        // Step 3: PCL 高度过滤
        pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
        pcl::fromROSMsg(cloud_2d, cloud_pcl);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_pcl.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*filtered);

        // Step 4: 投影到 LaserScan
        std::vector<float> ranges(num_readings_, std::numeric_limits<float>::infinity());

        for (const auto &p : filtered->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

            float range = std::hypot(p.x, p.y);
            if (range < min_range_ || range > max_range_) continue;

            float angle = std::atan2(p.y, p.x);

            // 归一化角度到 [angle_min, angle_max]
            while (angle < angle_min_) angle += 2 * M_PI;
            while (angle > angle_max_) angle -= 2 * M_PI;
            if (angle < angle_min_ || angle > angle_max_) continue;

            int idx = static_cast<int>(std::round((angle - angle_min_) / angle_inc_));
            if (idx < 0 || idx >= num_readings_) continue;

            if (range < ranges[idx]) {
                ranges[idx] = range;
            }
        }

        // Step 5: 发布
        auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = msg->header.stamp;
        scan->header.frame_id = output_frame_;
        scan->angle_min = angle_min_;
        scan->angle_max = angle_max_;
        scan->angle_increment = angle_inc_;
        scan->scan_time = scan_time_;
        scan->range_min = min_range_;
        scan->range_max = max_range_;
        scan->ranges = ranges;
        scan->intensities.resize(num_readings_, 0.0f);

        scan_pub_->publish(std::move(scan));
    }

    // 成员变量
    std::string pc_topic_, laserscan_topic_, lidar_frame_, base_frame_, stable_frame_, output_frame_;
    double min_height_, max_height_, min_range_, max_range_, scan_time_;
    double angle_min_, angle_max_, angle_inc_;
    int num_readings_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserScanForBody>());
    rclcpp::shutdown();
    return 0;
}