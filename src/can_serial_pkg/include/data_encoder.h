#ifndef SERIAL_ENCODER_HPP
#define SERIAL_ENCODER_HPP

#include <stdint.h>

#define INT14_MAX (0x1FFF)

// 串口数据帧格式
typedef struct down_ward_data
{
    // 2bytes
    int8_t chassis_x; // 底盘x方向速度，正方向是向前。 -INT8_MAX, INT8_MAX 对应 -1.5m/s , 1.5m/s
    int8_t chassis_y; // 底盘y方向速度，正方向是向左。-INT8_MAX, INT8_MAX 对应 -1.5m/s , 1.5m/s

    // 1byte
    int8_t chassis_yaw; // 底盘yaw方向角速度，正方向是向左。 -INT8_MAX, INT8_MAX 对应 -5rad/s , 5rad/s

    // 1byte
    int8_t gimbal_delta_pitch; // 云台pitch轴角度，-INT8_MAX, INT8_MAX 对应 -0.3 rad，0.3 rad

    // 2bytes
    int16_t gimbal_shoot : 1; // 云台发射
    int16_t gimbal_found : 1;
    int16_t gimbal_delta_yaw : 14; // 云台yaw轴delta角度, -(INT14_MAX), (INT14_MAX) 对应 -1.0 rad, 1.0 rad

} down_ward_data;

#define C_DATA_INT16_OFFSET 4

// 待发送的原始数据，从其他线程计算得到的浮点类型云台转角、底盘速度、整数类型的发射信息
typedef struct down_ward_data_raw_float
{
    float chassis_x;          // 底盘x方向速度
    float chassis_y;          // 底盘y方向速度
    float chassis_yaw;        // 底盘yaw方向角速度
    float gimbal_delta_yaw;   // 云台yaw轴角度
    float gimbal_delta_pitch; // 云台pitch轴角度
    int8_t gimbal_shoot;
    int8_t gimbal_found;
} down_ward_data_raw_float;

#define LIMIT_INSIDE(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min)
#define PI 3.14159265358979323846f

void encode_down_ward_data(const down_ward_data_raw_float *input, down_ward_data *output)
{

    output->chassis_x = LIMIT_INSIDE(MAP(input->chassis_x, -1.5, 1.5, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    output->chassis_y = LIMIT_INSIDE(MAP(input->chassis_y, -1.5, 1.5, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    output->chassis_yaw = LIMIT_INSIDE(MAP(input->chassis_yaw, -5.0, 5.0, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    output->gimbal_shoot = input->gimbal_shoot ? 1 : 0;
    output->gimbal_found = input->gimbal_found ? 1 : 0;
    output->gimbal_delta_yaw = LIMIT_INSIDE(MAP(input->gimbal_delta_yaw, -1.0, 1.0, -INT14_MAX, INT14_MAX), -INT14_MAX, INT14_MAX);
    output->gimbal_delta_pitch = LIMIT_INSIDE(MAP(input->gimbal_delta_pitch, -0.3, 0.3, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
}

void decode_down_ward_data(const down_ward_data *input, down_ward_data_raw_float *output)
{

    output->chassis_x = MAP(input->chassis_x, -INT8_MAX, INT8_MAX, -1.5, 1.5);
    output->chassis_y = MAP(input->chassis_y, -INT8_MAX, INT8_MAX, -1.5, 1.5);
    output->chassis_yaw = MAP(input->chassis_yaw, -INT8_MAX, INT8_MAX, -5, 5);
    output->gimbal_shoot = input->gimbal_shoot;
    output->gimbal_found = input->gimbal_found;
    output->gimbal_delta_yaw = MAP(input->gimbal_delta_yaw, -INT14_MAX, INT14_MAX, -1.0, 1.0);
    output->gimbal_delta_pitch = MAP(input->gimbal_delta_pitch, -INT8_MAX, INT8_MAX, -0.3, 0.3);
}

#endif