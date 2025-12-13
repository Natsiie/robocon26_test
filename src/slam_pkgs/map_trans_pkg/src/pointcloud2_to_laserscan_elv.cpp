#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <limits>

class PointCloud2ToLaserScanELV : public rclcpp::Node
{
public:
    PointCloud2ToLaserScanELV() : Node("pointcloud2_to_laserscan_elv")
    {
        // 声明参数
        this->declare_parameter("pc_topic", "/cloud_registered");
        this->declare_parameter("laserscan_topic", "/scan");
        this->declare_parameter("pgm_topic", "");
        this->declare_parameter("elv_topic", "");
        this->declare_parameter("min_height", -0.2);
        this->declare_parameter("max_height", 0.2);
        this->declare_parameter("min_dist", 0.1);
        this->declare_parameter("max_dist", 10.0);
        this->declare_parameter("angle_resolution_deg", 1.0);
        this->declare_parameter("angle_min_deg", -180.0);
        this->declare_parameter("angle_max_deg", 180.0);
        this->declare_parameter("passable_thresh", 0.1); // 可通行高度阈值
        this->declare_parameter("grid_resolution", 0.1); // PGM网格分辨率
        this->declare_parameter("elv_ratio", 2.0);       // 高程图与PGM比例
        this->declare_parameter("filter_kernel", 5);     // 滤波核大小
        this->declare_parameter("top_k", 5);             // 取最高K个点

        // 获取参数
        std::string pc_topic = this->get_parameter("pc_topic").as_string();
        std::string laserscan_topic = this->get_parameter("laserscan_topic").as_string();
        pgm_topic_ = this->get_parameter("pgm_topic").as_string();
        elv_topic_ = this->get_parameter("elv_topic").as_string();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        min_dist_ = this->get_parameter("min_dist").as_double();
        max_dist_ = this->get_parameter("max_dist").as_double();
        double angle_resolution_deg = this->get_parameter("angle_resolution_deg").as_double();
        double angle_min_deg = this->get_parameter("angle_min_deg").as_double();
        double angle_max_deg = this->get_parameter("angle_max_deg").as_double();
        passable_thresh_ = this->get_parameter("passable_thresh").as_double();
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        elv_ratio_ = this->get_parameter("elv_ratio").as_double();
        filter_kernel_ = this->get_parameter("filter_kernel").as_int();
        top_k_ = this->get_parameter("top_k").as_int();

        // 转换为弧度
        angle_resolution_ = angle_resolution_deg * M_PI / 180.0;
        angle_min_ = angle_min_deg * M_PI / 180.0;
        angle_max_ = angle_max_deg * M_PI / 180.0;

        // 计算角度范围内的点数
        num_measurements_ = static_cast<int>((angle_max_ - angle_min_) / angle_resolution_) + 1;

        // 创建订阅者
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloud2ToLaserScanELV::pointcloud_callback, this, std::placeholders::_1));

        // 创建发布者
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            laserscan_topic,
            rclcpp::SensorDataQoS());

        // 可选的地图和高程图发布者
        if (!pgm_topic_.empty())
        {
            pgm_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                pgm_topic_, rclcpp::SensorDataQoS());
            RCLCPP_INFO(this->get_logger(), "Publishing PGM map to: %s", pgm_topic_.c_str());
        }

        if (!elv_topic_.empty())
        {
            elv_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                elv_topic_, rclcpp::SensorDataQoS());
            RCLCPP_INFO(this->get_logger(), "Publishing elevation map to: %s", elv_topic_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "PointCloud2 to LaserScan ELV node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", pc_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", laserscan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Height range: [%.2f, %.2f] m", min_height_, max_height_);
        RCLCPP_INFO(this->get_logger(), "Distance range: [%.2f, %.2f] m", min_dist_, max_dist_);
        RCLCPP_INFO(this->get_logger(), "Angle resolution: %.2f degrees", angle_resolution_deg);
        RCLCPP_INFO(this->get_logger(), "Angle range: [%.2f, %.2f] degrees", angle_min_deg, angle_max_deg);
        RCLCPP_INFO(this->get_logger(), "Passable threshold: %.2f m", passable_thresh_);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = this->now();

        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // 1. 高度过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*height_filtered);

        if (height_filtered->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points after height filtering");
            return;
        }

        // 2. 计算点云范围
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*height_filtered, min_pt, max_pt);

        double x_min = min_pt.x;
        double y_min = min_pt.y;
        double x_max = max_pt.x;
        double y_max = max_pt.y;

        // 3. 构建高程网格
        auto elv_grid = build_elevation_grid(height_filtered, x_min, y_min, x_max, y_max);

        if (elv_grid.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to build elevation grid");
            return;
        }

        // 4. 分类可通行区域
        auto [pgm_grid, pgm_width, pgm_height] = classify_grid(elv_grid);

        // 5. 发布地图和高程图（如果配置了）
        if (!pgm_topic_.empty() && pgm_pub_)
        {
            publish_pgm_map(pgm_grid, pgm_width, pgm_height, x_min, y_min, msg->header);
        }

        if (!elv_topic_.empty() && elv_pub_)
        {
            publish_elevation_map(elv_grid, x_min, y_min, msg->header);
        }

        // 6. 生成LaserScan - 只在不可通行区域
        generate_laserscan_from_obstacles(elv_grid, pgm_grid, x_min, y_min, msg->header);

        auto end_time = this->now();
        auto duration = end_time - start_time;
        RCLCPP_DEBUG(this->get_logger(), "Processing time: %.3f seconds", duration.seconds());
    }

    // 构建高程网格
    cv::Mat build_elevation_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 double x_min, double y_min, double x_max, double y_max)
    {
        // 计算网格尺寸
        int elv_width = static_cast<int>((x_max - x_min) / (grid_resolution_ / elv_ratio_));
        int elv_height = static_cast<int>((y_max - y_min) / (grid_resolution_ / elv_ratio_));

        if (elv_width <= 0 || elv_height <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid elevation grid dimensions");
            return cv::Mat();
        }

        // 创建高程网格，初始化为负无穷
        cv::Mat elv_grid(elv_height, elv_width, CV_32F, cv::Scalar(-std::numeric_limits<float>::infinity()));

        // 网格分辨率
        double cell_res = grid_resolution_ / elv_ratio_;

        // 为每个点计算网格位置
        for (const auto &point : cloud->points)
        {
            double dist = std::sqrt(point.x * point.x + point.y * point.y);
            if (dist < min_dist_ || dist > max_dist_)
                continue;

            int ix = static_cast<int>((point.x - x_min) / cell_res);
            int iy = static_cast<int>((point.y - y_min) / cell_res);

            if (ix >= 0 && ix < elv_width && iy >= 0 && iy < elv_height)
            {
                float &current_height = elv_grid.at<float>(iy, ix);
                if (std::isinf(current_height) || point.z > current_height)
                {
                    current_height = point.z;
                }
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Built elevation grid: %d x %d", elv_width, elv_height);
        return elv_grid;
    }

    // 分类可通行区域
    std::tuple<cv::Mat, int, int> classify_grid(const cv::Mat &elv_grid)
    {
        if (elv_grid.empty())
        {
            return {cv::Mat(), 0, 0};
        }

        // 创建有效点掩码
        cv::Mat valid_mask = (elv_grid != -std::numeric_limits<float>::infinity());

        // 转换为float32进行计算
        cv::Mat elv_float;
        elv_grid.convertTo(elv_float, CV_32F);

        // 使用OpenCV进行形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size(filter_kernel_, filter_kernel_));

        // 计算局部最大值和最小值
        cv::Mat max_map, min_map;
        cv::dilate(elv_float, max_map, kernel);
        cv::erode(elv_float, min_map, kernel);

        // 计算高度差
        cv::Mat height_diff = max_map - min_map;

        // 计算有效点数量
        cv::Mat count_map;
        cv::filter2D(valid_mask, count_map, -1, kernel);

        // 未知区域掩码（有效点太少）
        double unknown_threshold = filter_kernel_ * filter_kernel_ * 0.2;
        cv::Mat unknown_mask = (count_map < unknown_threshold);

        // 创建分类结果
        cv::Mat pgm_full = cv::Mat::zeros(elv_grid.size(), CV_8U);

        // 可通行区域：高度差小于阈值
        cv::Mat passable_mask = (height_diff <= passable_thresh_) & ~unknown_mask;
        pgm_full.setTo(255, passable_mask); // 白色表示可通行

        // 未知区域：灰色
        pgm_full.setTo(127, unknown_mask);

        // 不可通行区域：黑色（默认为0）

        // 下采样到PGM尺寸
        int pgm_width = elv_grid.cols / static_cast<int>(elv_ratio_);
        int pgm_height = elv_grid.rows / static_cast<int>(elv_ratio_);

        cv::Mat pgm;
        cv::resize(pgm_full, pgm, cv::Size(pgm_width, pgm_height), 0, 0, cv::INTER_NEAREST);

        RCLCPP_DEBUG(this->get_logger(), "Classified grid: PGM size %d x %d", pgm_width, pgm_height);
        return {pgm, pgm_width, pgm_height};
    }

    // 生成LaserScan - 只在不可通行区域
    void generate_laserscan_from_obstacles(const cv::Mat &elv_grid, const cv::Mat &pgm_grid,
                                           double x_min, double y_min, std_msgs::msg::Header header)
    {
        if (elv_grid.empty() || pgm_grid.empty())
            return;

        // 创建范围数组，初始化为无穷大（表示没有障碍物）
        std::vector<double> ranges(num_measurements_, std::numeric_limits<double>::infinity());

        // 网格分辨率
        double cell_res = grid_resolution_ / elv_ratio_;

        // 遍历高程网格，找到不可通行区域
        for (int i = 0; i < elv_grid.rows; ++i)
        {
            for (int j = 0; j < elv_grid.cols; ++j)
            {
                if (std::isinf(elv_grid.at<float>(i, j)))
                    continue;

                // 计算世界坐标
                double x = x_min + j * cell_res;
                double y = y_min + i * cell_res;
                double range = std::sqrt(x * x + y * y);
                double angle = std::atan2(y, x);

                // 检查是否在距离范围内
                if (range < min_dist_ || range > max_dist_)
                    continue;

                // 计算对应的PGM网格位置
                int pgm_j = static_cast<int>(j / elv_ratio_);
                int pgm_i = static_cast<int>(i / elv_ratio_);

                if (pgm_j >= 0 && pgm_j < pgm_grid.cols && pgm_i >= 0 && pgm_i < pgm_grid.rows)
                {
                    uint8_t cell_value = pgm_grid.at<uint8_t>(pgm_i, pgm_j);

                    // 只在不可通行区域（黑色，值为0）生成LaserScan
                    if (cell_value == 0)
                    {
                        // 确保角度在范围内
                        if (angle < angle_min_)
                            angle += 2 * M_PI;
                        if (angle > angle_max_)
                            angle -= 2 * M_PI;

                        if (angle >= angle_min_ && angle <= angle_max_)
                        {
                            int index = static_cast<int>((angle - angle_min_) / angle_resolution_);
                            if (index >= 0 && index < num_measurements_)
                            {
                                // 保留最近的障碍物
                                if (range < ranges[index])
                                {
                                    ranges[index] = range;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 创建LaserScan消息
        sensor_msgs::msg::LaserScan scan;
        scan.header = header;
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_resolution_;
        scan.time_increment = 0.0;
        scan.scan_time = 1.0 / 30.0; // 假设30Hz
        scan.range_min = min_dist_;
        scan.range_max = max_dist_;

        // 设置范围值
        scan.ranges.resize(num_measurements_);
        for (int i = 0; i < num_measurements_; ++i)
        {
            if (ranges[i] < std::numeric_limits<double>::infinity())
            {
                scan.ranges[i] = static_cast<float>(ranges[i]);
            }
            else
            {
                // 无穷大表示没有障碍物（可通行或未知区域）
                scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        // 发布LaserScan消息
        scan_pub_->publish(scan);
    }

    // 发布PGM地图
    void publish_pgm_map(const cv::Mat &pgm_grid, int width, int height,
                         double origin_x, double origin_y, std_msgs::msg::Header header)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header = header;
        map_msg.info.resolution = grid_resolution_;
        map_msg.info.width = static_cast<uint32_t>(width);
        map_msg.info.height = static_cast<uint32_t>(height);
        map_msg.info.origin.position.x = origin_x;
        map_msg.info.origin.position.y = origin_y;
        map_msg.info.origin.orientation.w = 1.0;

        // 转换PGM网格到OccupancyGrid格式
        map_msg.data.resize(width * height);
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                uint8_t value = pgm_grid.at<uint8_t>(i, j);
                int index = i * width + j;

                if (value == 255)
                { // 可通行
                    map_msg.data[index] = 0;
                }
                else if (value == 127)
                { // 未知
                    map_msg.data[index] = -1;
                }
                else
                { // 不可通行
                    map_msg.data[index] = 100;
                }
            }
        }

        pgm_pub_->publish(map_msg);
    }

    // 发布高程图
    void publish_elevation_map(const cv::Mat &elv_grid, double origin_x, double origin_y,
                               std_msgs::msg::Header header)
    {
        if (elv_grid.empty())
            return;

        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header = header;
        map_msg.info.resolution = grid_resolution_ / elv_ratio_;
        map_msg.info.width = static_cast<uint32_t>(elv_grid.cols);
        map_msg.info.height = static_cast<uint32_t>(elv_grid.rows);
        map_msg.info.origin.position.x = origin_x;
        map_msg.info.origin.position.y = origin_y;
        map_msg.info.origin.orientation.w = 1.0;

        // 转换高程网格到OccupancyGrid格式
        map_msg.data.resize(elv_grid.cols * elv_grid.rows);

        float min_height = std::numeric_limits<float>::max();
        float max_height = -std::numeric_limits<float>::max();

        // 找到有效高度范围
        for (int i = 0; i < elv_grid.rows; ++i)
        {
            for (int j = 0; j < elv_grid.cols; ++j)
            {
                float height = elv_grid.at<float>(i, j);
                if (!std::isinf(height))
                {
                    min_height = std::min(min_height, height);
                    max_height = std::max(max_height, height);
                }
            }
        }

        if (max_height - min_height < 0.001)
        {
            max_height = min_height + 1.0; // 避免除零
        }

        // 归一化到0-100
        for (int i = 0; i < elv_grid.rows; ++i)
        {
            for (int j = 0; j < elv_grid.cols; ++j)
            {
                float height = elv_grid.at<float>(i, j);
                int index = i * elv_grid.cols + j;

                if (std::isinf(height))
                {
                    map_msg.data[index] = -1; // 未知
                }
                else
                {
                    int occupancy = static_cast<int>(100 * (height - min_height) / (max_height - min_height));
                    occupancy = std::clamp(occupancy, 0, 100);
                    map_msg.data[index] = static_cast<int8_t>(occupancy);
                }
            }
        }

        elv_pub_->publish(map_msg);
    }

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pgm_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elv_pub_;

    // 参数
    double min_height_;
    double max_height_;
    double min_dist_;
    double max_dist_;
    double angle_resolution_;
    double angle_min_;
    double angle_max_;
    int num_measurements_;
    double passable_thresh_;
    double grid_resolution_;
    double elv_ratio_;
    int filter_kernel_;
    int top_k_;
    std::string pgm_topic_;
    std::string elv_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloud2ToLaserScanELV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}