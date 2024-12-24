#ifndef MICK_CHASSIS_PROTOCOL_HPP
#define MICK_CHASSIS_PROTOCOL_HPP

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

// #include "serial/serial.h"

#define WHEEL_RATIO              (19.0)  // 麦克纳母轮模式 减速比 3508电机减速比为1:19
#define WHEEL_K                  (0.355) // 麦克纳母轮模式

#define WHEEL_L                  (0.4)   // 左右轮子的间距
#define WHEEL_D                  (0.17)  // 轮子直径  6.5寸的轮子
// #define  WHEEL_R    WHEEL_D/2.0 			//轮子半径
#define WHEEL_PI                 (3.141693) // pi

// 协议为大端模式, 高位低地址
//  ----------------------- 协议固定数据 -----------------------
#define FRAME_HEADER1            (0xAE) // 帧头标识
#define FRAME_HEADER2            (0xEA) // 帧头标识（2字节，0xAEEA）
#define FRAME_FOOTER1            (0xEF) // 帧尾标识
#define FRAME_FOOTER2            (0xFE) // 帧尾标识（2字节，0xEFFE）
#define MAX_DATA_LENGTH          (0xF)  // 最大数据长度(15字节)
#define CHECKSUM_SIZE            (1)    // 校验位（1字节）
#define FRAME_DATA_LEN_LOC       (2)    // 数据长度位置 2 个偏移量
// ----------------------- 命令类型定义 -----------------------
#define CMD_TYPE_DIFF_MOTO_DATA  (0x07) // 4轮差速底盘-电机数据（MickV3）
#define CMD_TYPE_DIFF_MOTO_STATE (0x08) // 4轮差速底盘-电机状态（MickV3）
#define CMD_TYPE_RC_DATA         (0xA3) // 遥控器数据
#define CMD_TYPE_ODOM_VEL        (0xA7) // 里程计之速度信息
#define CMD_TYPE_GPIO            (0xAC) // GPIO
// ----------------------- 接收状态机 -----------------------
typedef enum
{
    STATE_WAIT_FOR_HEADER, // 等待帧头
    STATE_READING_FRAME,   // 读取帧内容
    STATE_ERROR            // 错误状态
} mcpf_parser_state;       // mick chassis frame parser state

// ----------------------- 宏函数 -----------------------
#define FRAME_MINIMUM_SIZE (7)           // when data length == 0
#define FRAME_SIZE(D_LEN)  ((D_LEN) + 4) // data length + frame header length + frame footer length

void send_speed_to_chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, int chassis_type, float speed_x, float speed_y, float speed_w);

void send_rpm_to_chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, int w1, int w2, int w3, int w4);

void send_speed_to_X4chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x, float y, float w);

void send_speed_to_4WS4WDchassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x, float y, float w);

void send_speed_to_Ackerchassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x, float w);

void send_rpm_to_4WS4WDchassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, std::vector<float> vw);

void clear_odometry_chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd);

#endif
