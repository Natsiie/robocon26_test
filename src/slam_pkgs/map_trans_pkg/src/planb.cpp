#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <atomic>
#include <rclcpp/qos.hpp>


class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter() : Node("point_cloud_filter") {
        // 1. 声明参数 (带默认值)
        // 你可以在 launch 文件或命令行里修改这些值
        this->declare_parameter("angle0", 0.0);
        this->declare_parameter("angle1", 0.0);
        this->declare_parameter("angle2", 0.0);
        this->declare_parameter("angle3", 0.0);

        // 2. 订阅雷达点云
        // 改回默认的 10 (即 Reliable + Volatile)，与发布者完美匹配
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&PointCloudFilter::cloudCallback, this, std::placeholders::_1));

        // 3. 发布过滤后的点云
        filtered1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points1", 1);
        filtered2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points2", 1);
        
        // 4. 发布当前使用的角度 (方便调试查看)
        backboard_pub_ = this->create_publisher<std_msgs::msg::Float32>("/backboard_angle", 1);
        car_pub_ = this->create_publisher<std_msgs::msg::Float32>("/car_angle", 1);
        
        RCLCPP_INFO(this->get_logger(), "PointCloudFilter initialized. Waiting for pointcloud...");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // 1. 实时获取参数值 (支持动态调参)
        // 这样你在运行时用 ros2 param set 修改，这里马上就能生效
        float angle0 = this->get_parameter("angle0").as_double();
        float angle1 = this->get_parameter("angle1").as_double();
        float angle2 = this->get_parameter("angle2").as_double();
        float angle3 = this->get_parameter("angle3").as_double();

        // 2. 筛选 backboard_angle（优先级：angle1 > angle0）
        float backboard_angle = angle0; 
        if (std::abs(angle1) > 0.001) { // 如果 angle1 不为0 (浮点数比较)
            backboard_angle = angle1;
        }

        // 3. 筛选 car_angle（优先级：angle3 > angle2）
        float car_angle = angle2; 
        if (std::abs(angle3) > 0.001) { 
            car_angle = angle3;
        }

        // 4. 打印调试信息 (频率太高可以注释掉)
        RCLCPP_INFO(this->get_logger(), "Processing: backboard_angle=%.2f, car_angle=%.2f", backboard_angle, car_angle);

        // 5. 发布当前使用的角度
        auto backboard_angle_msg = std_msgs::msg::Float32();
        backboard_angle_msg.data = backboard_angle;
        backboard_pub_->publish(backboard_angle_msg);

        auto car_angle_msg = std_msgs::msg::Float32();
        car_angle_msg.data = car_angle;
        car_pub_->publish(car_angle_msg);

        // 6. 转换点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cartesian_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cartesian_cloud);

        if (cartesian_cloud->empty()) return;

        // 7. 高度过滤参数 (Z轴)
        // Filter 1 (Backboard): 0.45m ~ 1.7m
        float height_min1 = 0.55; 
        float height_max1 = 4.0;  
        // Filter 2 (Car): 0.001m ~ 0.4m
        float height_min2 = 0.001; 
        float height_max2 = 0.2;  

        // 8. 执行高度过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass1;
        pass1.setInputCloud(cartesian_cloud);
        pass1.setFilterFieldName("z");
        pass1.setFilterLimits(height_min1, height_max1);
        pass1.filter(*height_filtered_cloud1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass2;
        pass2.setInputCloud(cartesian_cloud);
        pass2.setFilterFieldName("z");
        pass2.setFilterLimits(height_min2, height_max2);
        pass2.filter(*height_filtered_cloud2);

        // 9. 坐标转换 lambda 函数 (笛卡尔 -> 球坐标)
        auto convertToSpherical = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr spherical_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            spherical_cloud->points.reserve(input_cloud->points.size());
            for (const auto& point : input_cloud->points) {
                float x = point.x;
                float y = point.y;
                float z = point.z;
                float radius = std::sqrt(x * x + y * y + z * z);
                float azimuth = std::atan2(x, y) * 180.0 / M_PI; // y轴为0度，x轴为90度
                float elevation = std::atan2(z, std::sqrt(x * x + y * y)) * 180.0 / M_PI;
                spherical_cloud->points.emplace_back(radius, azimuth, elevation);
            }
            return spherical_cloud;
        };

        // 10. 坐标转换 lambda 函数 (球坐标 -> 笛卡尔)
        auto convertToCartesian = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cartesian_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cartesian_cloud->points.reserve(input_cloud->points.size());
            for (const auto& point : input_cloud->points) {
                float radius = point.x;
                float azimuth = point.y * M_PI / 180.0;
                float elevation = point.z * M_PI / 180.0;
                float y = radius * std::cos(elevation) * std::cos(azimuth);
                float x = radius * std::cos(elevation) * std::sin(azimuth);
                float z = radius * std::sin(elevation);
                cartesian_cloud->points.emplace_back(x, y, z);
            }
            return cartesian_cloud;
        };

        // 11. 执行角度过滤
        auto spherical_cloud1 = convertToSpherical(height_filtered_cloud1);
        auto spherical_cloud2 = convertToSpherical(height_filtered_cloud2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr angle_filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        float min_angle1 = backboard_angle - 45.0;
        float max_angle1 = backboard_angle + 45.0;
        
        for (const auto& point : spherical_cloud1->points) {
            if (point.y >= min_angle1 && point.y <= max_angle1) {
                angle_filtered_cloud1->points.push_back(point);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr angle_filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        float min_angle2 = car_angle - 45.0;
        float max_angle2 = car_angle + 45.0;

        for (const auto& point : spherical_cloud2->points) {
            if (point.y >= min_angle2 && point.y <= max_angle2) {
                angle_filtered_cloud2->points.push_back(point);
            }
        }

        // 12. 转回笛卡尔坐标并发布
        auto final_cloud1 = convertToCartesian(angle_filtered_cloud1);
        auto final_cloud2 = convertToCartesian(angle_filtered_cloud2);

        auto output_msg1 = sensor_msgs::msg::PointCloud2();
        pcl::toROSMsg(*final_cloud1, output_msg1);
        output_msg1.header = cloud_msg->header;
        filtered1_pub_->publish(output_msg1);

        auto output_msg2 = sensor_msgs::msg::PointCloud2();
        pcl::toROSMsg(*final_cloud2, output_msg2);
        output_msg2.header = cloud_msg->header;
        filtered2_pub_->publish(output_msg2);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr backboard_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr car_pub_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}