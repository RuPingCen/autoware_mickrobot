/**
 * @file mick_robot_vehicle_interface.hpp
 * @brief mick_robot 底盘接入 autoware 接口
 * @author wyongcheng wyongcheng2social@foxmail.com
 * @version 1.0
 * @date 2024-12-24
 * @copyright Copyright (c) 2024
 *
 * 作者:   wyongcheng  修改日期:   2024-12-24    版本:   1.0
 */
#include "mick_chassis_msg.h"
#include "mick_chassis_protocol.hpp"
// #include <async_serial/BufferedAsyncSerial.h>
// #include "serial/serial.h"
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

// 0 : Differential, 1 : Mecanum, 2 : Ackermann, 3 : 4WS4WD
#define CHASSIS_DIFF       (0)
#define CHASSIS_MECANUM    (1)
#define CHASSIS_ACKERMANN  (2)
#define CHASSIS_4WS4WD     (3)
#define SERIAL_BUFFER_SIZE (500)
class MickRobotVehicleInterface : public rclcpp::Node
{
  public:
    using ControlModeCommand      = autoware_vehicle_msgs::srv::ControlModeCommand;
    using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
    MickRobotVehicleInterface();

  private:
    // ...
    // 必要信息 车速 档位 手动/自动状态
    /* subscribers */
    // From autoware
    // 转向、速度、加速度
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
    // 紧急停车
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;
    // 档位控制
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
    // 转向灯 假
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;
    // 双闪 危险报警闪光灯
    rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;
    // 油门 刹车
    // rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;

    /* publishers */
    // To autoware
    // 车辆档位状态
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
    // 上报车辆控制模式 用于确定是否被人工接管
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
    // 上报车速
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
    // 上报转向
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
    // 上报转向灯
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_status_pub_;
    // 上报双闪灯
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_status_pub_;
    // 上报油门 刹车
    // rclcpp::Publisher<ActuationStatusStamped>::SharedPtr                           actuation_status_pub_;
    // 车门状态
    //  rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr                  door_status_pub_;

    /* Form and to mick_chassis, serial Read and Write */
    // BufferedAsyncSerial *mick_chassis_serial_;
    // serial::Serial                            mick_chassis_serial_;
    std::shared_ptr<boost::asio::io_service>  io_service_;
    std::shared_ptr<boost::asio::serial_port> mick_chassis_serial_port_;

    /* timer used to publish */
    rclcpp::TimerBase::SharedPtr timer_;
    /* timer used to serial */
    rclcpp::TimerBase::SharedPtr serial_timer_;

    /* ros param */
    std::string cmdvel_stopic_; // autoware control command subcribe topic name
    std::string joy_ptopic_;    // joy stick information publish topic name
    // std::string pub_odom_topic_;
    // std::string pub_imu_topic_;
    std::string base_frame_id_;
    std::string serial_dev_;              // serial device port
    int         baud_;                    // Baud
    int         command_timeout_ms_;      // vehicle_cmd timeout [ms]
    int         loop_rate_;               //[Hz]
    int         chassis_type_;            // default to differential drive  0：Differential 1: Mecanum 2: Ackermann 3: 4WS4WD
    float       RC_K_;                    // 遥控器摇杆通道输出 比例系数
    float       RC_MIN_;                  // 遥控器摇杆通道输出 最小值
    float       RC_MAX_;                  // 遥控器摇杆通道输出 最大值

    double tire_radius_;                  // [m]
    double wheel_base_;                   // [m]
    double vehicle_length_;               // [m]
    double emergency_brake_;              // brake command when emergency [m/s^2]
    bool   use_external_emergency_brake_; // set to true to not use emergency_brake_

    autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
    /* Property */
    // 内部维护状态，假装给autoware换档 换灯
    uint8_t Gear_static;
    uint8_t TurnIndicatorsCommand_static;
    uint8_t HazardLights_static;

    // Service
    rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_server_;

    /* input value */
    // autoware command messages
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;

    // 串口读到的数据
    std_msgs::msg::String::SharedPtr serial_info_data_;
    // 串口写入的数据
    std_msgs::msg::String::SharedPtr serial_control_data_;
    // 底盘数据
    chassis_measure_t *chassis_measure_ptr;

    bool         engage_cmd_{false};
    bool         is_emergency_{false};
    rclcpp::Time control_command_received_time_;
    rclcpp::Time chassis_received_time_;
    // 用于数据解析转换
    // union的作用为实现char数组和int32之间的转换
    union INT32Data
    {
        int32_t       int32_dat;
        unsigned char byte_data[4];
    } motor_upload_counter_, total_angle_, round_cnt_, speed_rpm_;
    // union的作用为实现char数组和int16数据类型之间的转换
    union Int16Data
    {
        int16_t       int16_dat;
        unsigned char byte_data[2];
    } imu_, odom_;

    // callbacks
    void callbackControlCmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
    void callbackEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
    void callbackGearCmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
    void callbackTurnIndicatorsCommand(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
    void callbackHazardLightsCommand(const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);

    /*  functions */
    // 车的控制指令在这里通过串口发布
    void toVehiclepublishCommands();
    // 反馈给autoware的状态在这里通过话题发布
    void toAutowarepublishStatus();
    void onControlModeRequest(const ControlModeCommand::Request::SharedPtr request, const ControlModeCommand::Response::SharedPtr response);
    bool parseMikcRobotSerialData(std::string &str_data, chassis_measure_t *chassis_measure_ptr);
    bool validateFrame(uint8_t *frame, uint16_t frameLength);
};
