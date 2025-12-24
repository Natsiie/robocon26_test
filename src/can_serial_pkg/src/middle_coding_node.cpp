// can_serial_pkg/src/middle_coding_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"
#include "data_encoder.h"

class MiddleCodingNode : public rclcpp::Node
{
public:
    MiddleCodingNode() : Node("middle_coding_node")
    {
        // 订阅/cmd_vel话题，接收geometry_msgs/Twist消息
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&MiddleCodingNode::cmdVelCallback, this, std::placeholders::_1));
        
        // 发布编码后的数据到/msg_to_can话题
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/msg_to_can", 10);
        
        RCLCPP_INFO(this->get_logger(), "Middle Coding Node started. Subscribing to /cmd_vel and publishing to /msg_to_can");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 创建原始数据结构
        down_ward_data_raw_float raw_data;
        down_ward_data encoded_data;
        
        // 从Twist消息填充底盘数据
        raw_data.chassis_x = static_cast<float>(msg->linear.x);      // x方向线速度
        raw_data.chassis_y = static_cast<float>(msg->linear.y);      // y方向线速度
        raw_data.chassis_yaw = static_cast<float>(msg->angular.z);   // z方向角速度（yaw）
        
        // 初始化云台数据为0（如果需要从其他来源获取云台数据，可以在这里更新）
        raw_data.gimbal_delta_yaw = 0.0f;
        raw_data.gimbal_delta_pitch = 0.0f;
        raw_data.gimbal_shoot = 0;
        raw_data.gimbal_found = 0;
        
        // 编码数据
        encode_down_ward_data(&raw_data, &encoded_data);
        
        // 创建CAN消息
        auto can_frame_msg = can_msgs::msg::Frame();
        can_frame_msg.id = 5;  
        can_frame_msg.dlc = 8;
        
        // 将编码后的数据填充到CAN消息中
        // 注意：这里按照结构体内存布局进行复制
        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&encoded_data);
        for (int i = 0; i < 8 && (size_t)i < sizeof(down_ward_data); i++) {
            can_frame_msg.data[i] = data_ptr[i];
        }
        
        // 补零（如果有额外字节）
        for (int i = sizeof(down_ward_data); i < 8; i++) {
            can_frame_msg.data[i] = 0;
        }
        
        // 添加时间戳
        can_frame_msg.header.stamp = this->now();
        
        // 发布消息
        can_pub_->publish(can_frame_msg);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiddleCodingNode>());
    rclcpp::shutdown();
    return 0;
}