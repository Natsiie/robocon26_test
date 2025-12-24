// can_serial_pkg/src/can_serial_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

#include <libserialport.h>
#include "can_coder.hpp"

#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <cstring>

using namespace std::chrono_literals;

class SerialCanNode : public rclcpp::Node
{
public:
    explicit SerialCanNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("serial_can_node", options), port_(nullptr)
    {
        // 声明参数
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("can_downward_topic", "/msg_to_can");
        this->declare_parameter("can_upward_topic", "/msg_from_can");

        this->get_parameter("serial_port", serial_port_);
        this->get_parameter("baud_rate", baud_rate_);
        this->get_parameter("can_downward_topic", can_downward_topic_);
        this->get_parameter("can_upward_topic", can_upward_topic_);

        // 创建发布者和订阅者
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>(can_upward_topic_, 10);
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            can_downward_topic_, 10,
            std::bind(&SerialCanNode::canDownwardCallback, this, std::placeholders::_1));

        // 打开串口
        configureSerial();

        // 启动接收线程
        receiver_thread_ = std::thread(&SerialCanNode::receiverThread, this);

        RCLCPP_INFO(this->get_logger(), "Serial CAN node started: %s @ %d", serial_port_.c_str(), baud_rate_);
    }

    ~SerialCanNode()
    {
        if (receiver_thread_.joinable())
        {
            receiver_thread_.join();
        }
        closeSerial();
    }

private:
    void configureSerial()
    {
        closeSerial(); // 先确保旧端口已关闭

        enum sp_return ret = sp_get_port_by_name(serial_port_.c_str(), &port_);
        if (ret != SP_OK || port_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot find serial port: %s", serial_port_.c_str());
            return;
        }

        ret = sp_open(port_, SP_MODE_READ_WRITE);
        if (ret != SP_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            sp_free_port(port_);
            port_ = nullptr;
            return;
        }

        sp_set_baudrate(port_, baud_rate_);
        sp_set_bits(port_, 8);
        sp_set_parity(port_, SP_PARITY_NONE);
        sp_set_stopbits(port_, 1);
        sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s @ %d bps", serial_port_.c_str(), baud_rate_);
    }

    void closeSerial()
    {
        if (port_)
        {
            sp_close(port_);
            sp_free_port(port_);
            port_ = nullptr;
        }
    }

    void canDownwardCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if (!port_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Serial port not open, drop CAN frame to send");
            return;
        }

        uint8_t send_data[CAN_LEN_IN_BYTE];
        can_frame send_frame;
        send_frame.id = msg->id;
        send_frame.len = msg->dlc;
        memcpy(send_frame.data, msg->data.data(), CAN_DATA_IN_BYTE);

        encode_can_frame(&send_frame, send_data);
        RCLCPP_INFO(this->get_logger(),"send data : %s", hex2str_c(send_data));

        int written = sp_blocking_write(port_, send_data, CAN_LEN_IN_BYTE, 200); // 200ms超时
        if (written != CAN_LEN_IN_BYTE)
        {
            RCLCPP_WARN(this->get_logger(), "Serial write incomplete: %d/%d bytes", written, CAN_LEN_IN_BYTE);
        }
    }

    void receiverThread()
    {
        uint8_t buffer[CAN_LEN_IN_BYTE];
        while (rclcpp::ok())
        {
            if (!port_)
            {
                std::this_thread::sleep_for(1s);
                configureSerial();
                continue;
            }

            // 读帧头
            int n = sp_blocking_read(port_, buffer, 1, 100);
            if (n != 1 || buffer[0] != CAN_HEAD)
            {
                continue;
            }

            // 读剩余数据
            n = sp_blocking_read(port_, buffer + 1, CAN_LEN_IN_BYTE - 1, 100);
            if (n != CAN_LEN_IN_BYTE - 1)
            {
                RCLCPP_WARN(this->get_logger(), "Incomplete frame received (%d bytes)", n + 1);
                continue;
            }


            if (buffer[CAN_LEN_IN_BYTE - 1] != CAN_TAIL)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid frame tail at recv: 0x%02X", buffer[CAN_LEN_IN_BYTE - 1]);
                continue;
            }
            

            // RCLCPP_INFO(this->get_logger(),"recv data : %s", hex2str_c(buffer));

            // 解码并发布
            can_frame recv_frame;
            decode_can_frame(buffer, &recv_frame);

            auto frame_msg = can_msgs::msg::Frame();
            frame_msg.header.stamp = this->now();
            frame_msg.id = recv_frame.id;
            frame_msg.dlc = recv_frame.len;
            memcpy(frame_msg.data.data(), recv_frame.data, CAN_DATA_IN_BYTE);

            can_pub_->publish(frame_msg);
        }
    }

    const char* hex2str_c(const uint8_t* data, size_t len = CAN_LEN_IN_BYTE)
    {
        thread_local static char buf[256]; 
        char* p = buf;
        for (size_t i = 0; i < len; ++i)
        {
            if (i > 0) *p++ = ' ';
            p += sprintf(p, "%02X", data[i]);
        }
        *p = '\0';
        return buf;
    }

    // 成员变量
    struct sp_port *port_ = nullptr;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    std::string serial_port_;
    int baud_rate_;
    std::string can_downward_topic_;
    std::string can_upward_topic_;

    std::thread receiver_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCanNode>());
    rclcpp::shutdown();
    return 0;
}