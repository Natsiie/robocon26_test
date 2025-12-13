// src/pc2_to_livoxcloud.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <unordered_set>
#include <sstream>
#include <memory>

class PC2ToLivoxCloud : public rclcpp::Node
{
public:
    PC2ToLivoxCloud() : Node("pc2_to_livoxcloud")
    {
        this->declare_parameter<std::string>("input_topic", "/livox/lidar_pc2");
        this->declare_parameter<std::string>("output_topic", "/livox/lidar");

        std::string input = this->get_parameter("input_topic").as_string();
        std::string output = this->get_parameter("output_topic").as_string();

        pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input, 10,
            std::bind(&PC2ToLivoxCloud::callback, this, std::placeholders::_1));

        livox_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(output, 10);

        RCLCPP_INFO(this->get_logger(), "PC2 to Livox CustomMsg converter STARTED");
        RCLCPP_INFO(this->get_logger(), "  Input  : %s", input.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output : %s", output.c_str());

        // 添加字段日志标志，只打印一次
        first_message_ = true;
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->data.empty() || msg->point_step == 0 || msg->width == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Empty PointCloud2 received");
            return;
        }

        // 记录接收到的消息信息（只在第一次或字段变化时打印）
        if (first_message_)
        {
            // logPointCloudInfo(msg);
            first_message_ = false;
        }

        // 收集存在的字段
        std::unordered_set<std::string> existing_fields;
        for (const auto &field : msg->fields)
        {
            existing_fields.insert(field.name);
        }

        // 检查必需字段 (x, y, z)
        bool has_xyz = true;
        std::vector<std::string> missing_fields;

        if (existing_fields.find("x") == existing_fields.end())
        {
            missing_fields.push_back("x");
            has_xyz = false;
        }
        if (existing_fields.find("y") == existing_fields.end())
        {
            missing_fields.push_back("y");
            has_xyz = false;
        }
        if (existing_fields.find("z") == existing_fields.end())
        {
            missing_fields.push_back("z");
            has_xyz = false;
        }

        if (!has_xyz)
        {
            RCLCPP_ERROR(this->get_logger(), "Missing required fields: %s",
                         joinStrings(missing_fields, ", ").c_str());
            return;
        }

        // 可选字段
        bool has_intensity = existing_fields.find("intensity") != existing_fields.end();
        bool has_time = existing_fields.find("timestamp") != existing_fields.end();
        bool has_line = existing_fields.find("line") != existing_fields.end();

        // 记录缺失的可选字段
        if (!has_intensity)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "PointCloud2 missing intensity field, using default value 0");
        }

        std::string time_field_name = "timestamp";
        if (!has_time)
        {
            // 尝试其他可能的时间字段名称
            std::vector<std::string> possible_time_fields = {"time", "t", "stamp"};
            for (const auto &field_name : possible_time_fields)
            {
                if (existing_fields.find(field_name) != existing_fields.end())
                {
                    time_field_name = field_name;
                    has_time = true;
                    RCLCPP_INFO(this->get_logger(), "Using time field: %s", time_field_name.c_str());
                    break;
                }
            }

            if (!has_time)
            {
                RCLCPP_WARN_ONCE(this->get_logger(), "PointCloud2 missing time field, using header stamp");
            }
        }

        if (!has_line)
        {
            RCLCPP_DEBUG_ONCE(this->get_logger(), "PointCloud2 missing line field, using default value 0");
        }

        // 创建必需字段的迭代器
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        // 可选字段迭代器 - 使用指针在需要时才创建
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity_ptr;
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<double>> it_time_ptr;
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> it_line_ptr;

        if (has_intensity)
        {
            it_intensity_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
        }

        if (has_time)
        {
            it_time_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<double>>(*msg, time_field_name.c_str());
        }

        if (has_line)
        {
            it_line_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "line");
        }

        livox_ros_driver2::msg::CustomMsg livox_msg;
        livox_msg.header = msg->header;
        livox_msg.header.frame_id = "livox_frame";
        livox_msg.lidar_id = 0;
        livox_msg.point_num = msg->width * msg->height;
        livox_msg.points.resize(livox_msg.point_num);

        uint64_t timebase_ns = 0;
        bool first = true;

        size_t i = 0;
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++i)
        {
            uint64_t t_ns = 0;

            // 处理时间
            if (has_time && it_time_ptr)
            {
                t_ns = static_cast<uint64_t>((**it_time_ptr) * 1e9);
                ++(*it_time_ptr);
            }
            else
            {
                // 使用消息头的时间戳
                t_ns = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
            }

            if (first)
            {
                timebase_ns = t_ns;
                first = false;
            }

            livox_msg.points[i].x = *it_x;
            livox_msg.points[i].y = *it_y;
            livox_msg.points[i].z = *it_z;

            // 处理反射率
            if (has_intensity && it_intensity_ptr)
            {
                livox_msg.points[i].reflectivity = static_cast<uint8_t>(std::min(255.0f, **it_intensity_ptr));
                ++(*it_intensity_ptr);
            }
            else
            {
                livox_msg.points[i].reflectivity = 0;
            }

            livox_msg.points[i].tag = 0;
            livox_msg.points[i].offset_time = static_cast<uint32_t>(t_ns - timebase_ns);

            // 处理线号
            if (has_line && it_line_ptr)
            {
                livox_msg.points[i].line = static_cast<uint8_t>(**it_line_ptr);
                ++(*it_line_ptr);
            }
            else
            {
                livox_msg.points[i].line = 0;
            }
        }

        livox_msg.timebase = timebase_ns;
        livox_msg.rsvd[0] = livox_msg.rsvd[1] = livox_msg.rsvd[2] = 0;

        livox_pub_->publish(livox_msg);
    }

    // 记录点云信息
    void logPointCloudInfo(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::stringstream ss;
        ss << "PointCloud2 Info:" << std::endl;
        ss << "  Width: " << msg->width << std::endl;
        ss << "  Height: " << msg->height << std::endl;
        ss << "  Point count: " << msg->width * msg->height << std::endl;
        ss << "  Point step: " << msg->point_step << std::endl;
        ss << "  Row step: " << msg->row_step << std::endl;
        ss << "  Data size: " << msg->data.size() << " bytes" << std::endl;
        ss << "  Is dense: " << (msg->is_dense ? "Yes" : "No") << std::endl;
        ss << "  Header frame_id: " << msg->header.frame_id << std::endl;

        ss << "  Fields (" << msg->fields.size() << "):" << std::endl;
        for (size_t i = 0; i < msg->fields.size(); ++i)
        {
            const auto &field = msg->fields[i];
            ss << "    [" << i << "] " << field.name
               << " (offset: " << field.offset
               << ", datatype: " << field.datatype
               << ", count: " << field.count << ")" << std::endl;
        }

        // 检查常用字段是否存在
        std::vector<std::string> common_fields = {"x", "y", "z", "intensity", "timestamp", "time", "line"};
        ss << "  Common field status:" << std::endl;
        for (const auto &field_name : common_fields)
        {
            bool found = false;
            for (const auto &field : msg->fields)
            {
                if (field.name == field_name)
                {
                    found = true;
                    break;
                }
            }
            ss << "    " << field_name << ": " << (found ? "Present" : "Missing") << std::endl;
        }

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // 辅助函数：连接字符串
    std::string joinStrings(const std::vector<std::string> &strings, const std::string &delimiter)
    {
        std::string result;
        for (size_t i = 0; i < strings.size(); ++i)
        {
            result += strings[i];
            if (i < strings.size() - 1)
            {
                result += delimiter;
            }
        }
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub_;
    bool first_message_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PC2ToLivoxCloud>());
    rclcpp::shutdown();
    return 0;
}