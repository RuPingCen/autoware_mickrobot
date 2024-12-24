#include <mick_robot_vehicle_interface/mick_robot_vehicle_interface.hpp>
// fixme 这里的logger信息传的格式化类型不对，会报警告，找时间处理掉
// 如
/*   306 |             RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "frame %d, length %d, is valid %d", k,
   frame_len, |                                                                                                       ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  ~ | |
      |                                                                                                                                           size_t {aka
   long unsigned int}
 */
/*
  306 |             RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "frame %d, length %d, is valid %d", k, frame_len,
      |                                                                                                              ~^
      |                                                                                                               |
      |                                                                                                               int
      |                                                                                                              %ld
 */
MickRobotVehicleInterface::MickRobotVehicleInterface()
    : Node("mick_robot_vehicle_interface")
    , vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
    /* setup parameters */
    base_frame_id_      = declare_parameter("base_frame_id", "base_link");
    command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
    loop_rate_          = declare_parameter("loop_rate", 30.0);

    // parameters for vehicle description
    tire_radius_        = vehicle_info_.wheel_radius_m;
    wheel_base_         = vehicle_info_.wheel_base_m;

    emergency_brake_    = declare_parameter("emergency_brake", 0.7);
    serial_dev_         = declare_parameter("serial_dev", "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0");
    baud_               = declare_parameter("baud", 115200);
    joy_ptopic_         = declare_parameter("joy_ptopic", "/mick_robot/rc_remotes/joy");
    chassis_type_       = declare_parameter("chassis_type", CHASSIS_DIFF);
    RC_K_               = declare_parameter("rc_k", 0);
    RC_MIN_             = declare_parameter("rc_min", 0);
    RC_MAX_             = declare_parameter("rc_max", 0);

    RCLCPP_INFO_STREAM(get_logger(), "wheel base:      " << wheel_base_);
    RCLCPP_INFO_STREAM(get_logger(), "tire radius:     " << tire_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "RC_K:            " << RC_K_);
    RCLCPP_INFO_STREAM(get_logger(), "RC_MIN:          " << RC_MIN_);
    RCLCPP_INFO_STREAM(get_logger(), "RC_MAX:          " << RC_MAX_);

    engage_cmd_ = true; // CHENG 直接开始自动驾驶，不妥。需要在正式运行的时候更正，自动驾驶模式应有人工授权
    // 需先设置一下初值，否则判断超时，需要相减，若时钟源不匹配会直接挂掉
    control_command_received_time_ = this->now();
    chassis_received_time_         = this->now();
    Gear_static                    = autoware_vehicle_msgs::msg::GearReport::PARK;

    /* setup debug output */
    rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);

    /* subscribers */
    using std::placeholders::_1;
    using std::placeholders::_2;

    // From autoware
    control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 1,
                                                                                std::bind(&MickRobotVehicleInterface::callbackControlCmd, this, _1));
    gear_cmd_sub_    = create_subscription<autoware_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1,
                                                                                 std::bind(&MickRobotVehicleInterface::callbackGearCmd, this, _1));
    control_mode_server_ =
        create_service<ControlModeCommand>("input/control_mode_request", std::bind(&MickRobotVehicleInterface::onControlModeRequest, this, _1, _2));

    /* publisher */
    // To Autoware
    control_mode_pub_         = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});
    vehicle_twist_pub_        = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS{1});
    gear_status_pub_          = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});
    // steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});

    /* Serial */

    // From mick_chassis
    io_service_               = std::make_shared<boost::asio::io_service>();
    mick_chassis_serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);

    chassis_measure_ptr       = new chassis_measure_t();
    try
        {
            mick_chassis_serial_port_->open(serial_dev_);
    } catch (boost::system::system_error &error)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unable to open port." << serial_dev_ << error.what());
            throw;
    }
    if (mick_chassis_serial_port_->is_open())
        {
            RCLCPP_INFO_STREAM(get_logger(), "Serial Port " << serial_dev_ << " opened");
        }
    else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unable to open port." << serial_dev_);
            throw;
        }
    typedef boost::asio::serial_port_base sb;
    sb::baud_rate                         baud_option(baud_);
    sb::flow_control                      flow_control(sb::flow_control::none);
    sb::parity                            parity(sb::parity::none);
    sb::stop_bits                         stop_bits(sb::stop_bits::one);

    mick_chassis_serial_port_->set_option(baud_option);
    mick_chassis_serial_port_->set_option(flow_control);
    mick_chassis_serial_port_->set_option(parity);
    mick_chassis_serial_port_->set_option(stop_bits);

    /* Timer */
    const auto period_ns = rclcpp::Rate(loop_rate_).period();
    timer_               = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&MickRobotVehicleInterface::toVehiclepublishCommands, this));
    // 串口没有回调函数，只好写定时器循环了
    serial_timer_        = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&MickRobotVehicleInterface::toAutowarepublishStatus, this));
}

void MickRobotVehicleInterface::callbackEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
}

void MickRobotVehicleInterface::callbackGearCmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
    Gear_static = msg->command;
}
void MickRobotVehicleInterface::callbackControlCmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
    control_command_received_time_ = this->now();
    control_cmd_ptr_               = msg;
}

void MickRobotVehicleInterface::toVehiclepublishCommands()
{
    /* 保护 */
    if (!control_cmd_ptr_)
        {
            return;
        }

    const rclcpp::Time current_time        = get_clock()->now();

    // 将要下发的车速和转向角指令
    float desired_vx                       = control_cmd_ptr_->longitudinal.velocity;
    // 轮胎转向角转换成角速度
    float desired_wz                       = (desired_vx / wheel_base_) * control_cmd_ptr_->lateral.steering_tire_angle;

    /* check emergency and timeout */
    const double control_cmd_delta_time_ms = (current_time - control_command_received_time_).seconds() * 1000.0;
    bool         timeouted                 = false;
    const int    t_out                     = command_timeout_ms_;
    if (t_out >= 0 && (control_cmd_delta_time_ms > t_out))
        {
            timeouted = true;
        }
    /* check emergency and timeout */
    const bool emergency_brake_needed = (is_emergency_ && !use_external_emergency_brake_) || timeouted;
    if (emergency_brake_needed)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
                                  "Emergency Stopping, emergency = %d, control command timeouted = %d", is_emergency_, timeouted);
            desired_vx = 0;
            desired_wz = 0;
        }

    /* make engage cmd false when a driver overrides vehicle control */
    // todo 底盘控制覆盖
    // 和底盘上报信息有关，如果上报底盘正在操作，则engage_cmd_ = false，autoware退出控制
    // if (!prev_override_ && global_rpt_ptr_->override_active)
    //     {
    //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Pacmod is overridden, enable flag is back to false");
    //         engage_cmd_ = false;
    //     }
    // prev_override_              = global_rpt_ptr_->override_active;

    /* make engage cmd false when vehicle report is timed out, e.g. E-stop is depressed */
    const bool report_timed_out = ((current_time - chassis_received_time_).seconds() > 1.0);
    if (report_timed_out)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
                                 "mick chassis report is timed out, engage flag is back to false");
            engage_cmd_ = false;
        }

    /* make engage cmd false when vehicle fault is active */
    // todo 底盘状态异常处理
    // 需要小车上传状态数据
    // if (global_rpt_ptr_->pacmod_sys_fault_active)
    //     {
    //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Pacmod fault is active, enable flag is back to
    //         false"); engage_cmd_ = false;
    //     }
    // RCLCPP_DEBUG(get_logger(),
    //              "is_pacmod_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = "
    //              "%d",
    //              is_pacmod_enabled_, is_clear_override_needed_, clear_override);
    // cheng
    // fixme engage 这个处理不妥，engage应该是自动驾驶 接管/放弃 操控车辆，这时不应下发速度控制才对
    if (false == engage_cmd_)
        {
            desired_vx = 0;
            desired_wz = 0;
        }

    /* publish accel cmd */
    {
        float speed_x, speed_y, speed_w;
        speed_x = desired_vx;
        speed_y = 0;
        speed_w = desired_wz;
        if (Gear_static == autoware_vehicle_msgs::msg::GearReport::REVERSE)
            {
                speed_x = -speed_x;
            }
        else if (Gear_static == autoware_vehicle_msgs::msg::GearReport::PARK)
            {
                speed_x = 0;
                speed_w = 0;
            }
        else if (Gear_static == autoware_vehicle_msgs::msg::GearReport::DRIVE)
            {
                speed_x = desired_vx;
                speed_y = 0;
                speed_w = desired_wz;
            }
        else
            {
                speed_x = 0;
                speed_y = 0;
                speed_w = 0;
            }
        send_speed_to_chassis(mick_chassis_serial_port_, chassis_type_, speed_x, speed_y, speed_w);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), " send speed to chassis: speed_x = %f speed_w = %f", speed_x,
                             speed_w);
    }
    /* publish brake cmd */
    {
        // mick_chassis_serial_->writeString("Hello world\r\n");
    }
    /* publish steering cmd */
    {
        // mick_chassis_serial_->writeString("Hello world\r\n");
    }
}

void MickRobotVehicleInterface::toAutowarepublishStatus()
{
    uint8_t     serial_buf[1024];
    std::string serial_data_string;
    size_t      bytes_read = mick_chassis_serial_port_->read_some(boost::asio::buffer(serial_buf, 1024));
    if (bytes_read > 0)
        {
            for (size_t i = 0; i < bytes_read; ++i)
                {
                    serial_data_string.push_back(serial_buf[i]);
                }
            bool serial_data_valid = parseMikcRobotSerialData(serial_data_string, chassis_measure_ptr);
            if (serial_data_valid)
                {
                    chassis_received_time_ = this->now();
                    std_msgs::msg::Header header;
                    header.frame_id = base_frame_id_;
                    header.stamp    = chassis_received_time_;
                    /* publish vehicle status twist */
                    {
                        autoware_vehicle_msgs::msg::VelocityReport twist;
                        twist.header                = header;
                        twist.longitudinal_velocity = chassis_measure_ptr->odom_measurements.vx; // [m/s]
                        twist.heading_rate          = chassis_measure_ptr->odom_measurements.wz; // [rad/s]
                        vehicle_twist_pub_->publish(twist);
                    }
                    /* publish vehicle status control_mode */
                    {
                        autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
                        control_mode_msg.stamp = header.stamp;
                        // TODO 应该放在下面 if (1 == chassis_measure_ptr->rc.sw1)
                        // CHENG
                        control_mode_msg.mode  = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
                        control_mode_pub_->publish(control_mode_msg);
                    }
                    /* publish current shift */
                    {
                        autoware_vehicle_msgs::msg::GearReport gear_report_msg;
                        gear_report_msg.stamp  = header.stamp;
                        gear_report_msg.report = Gear_static;
                        gear_status_pub_->publish(gear_report_msg);
                    }

                    if (1 == chassis_measure_ptr->rc.sw1)
                        {
                            // swA位于左上角，小车请求切换自动驾驶模式
                            // engage_cmd_ = true;
                            // RCLCPP_WARN_STREAM(get_logger(), " mick chassis request auto drive mode, engage flag is set to true");
                        }

                    // delete chassis_measure_ptr;
                }
            else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "serial recive data error ...");
                    // delete chassis_measure_ptr;
                }
        }
}

void MickRobotVehicleInterface::onControlModeRequest(const ControlModeCommand::Request::SharedPtr  request,
                                                     const ControlModeCommand::Response::SharedPtr response)
{
    if (request->mode == ControlModeCommand::Request::AUTONOMOUS)
        {
            engage_cmd_       = true;
            response->success = true;
            return;
        }

    if (request->mode == ControlModeCommand::Request::MANUAL)
        {
            engage_cmd_       = false;
            response->success = true;
            return;
        }

    RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
    response->success = false;
    return;
}
// CHENG 见 note
/**
 * @brief 解析串口发送过来的数据帧
 * @param  str_data 缓冲区串口
 * @param  chassis_measure_ptr 解析返回结构体指针
 * @return true 数据有效
 * @return false 数据无效
 * @note 不太清楚为什么需要用两个for来解析数据，第一个找到帧头同时解析也可以？
 * @note
 */
bool MickRobotVehicleInterface::parseMikcRobotSerialData(std::string &str_data, chassis_measure_t *chassis_measure_ptr)
{
    bool              parse_result = false;
    unsigned char     rec_buffer[SERIAL_BUFFER_SIZE];
    mcpf_parser_state rec_state     = STATE_WAIT_FOR_HEADER; // 接收状态
    uint16_t          frame_num     = 0;                     // str_data 有 frame_num 个帧
    uint16_t          str_data_size = str_data.size();

    // check lenth
    if (str_data_size < FRAME_MINIMUM_SIZE || str_data_size > SERIAL_BUFFER_SIZE)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "serial data length out of bounds, len: %d",
                                 str_data.size());
            return parse_result;
        }
    // find frame header and parse data, use state machine
    unsigned char byte_last = 0, byte_curr = 0;       // 这是在 str_data 里滑行的字节
    for (size_t i = 0, j = 0; i < str_data_size; i++) // i,j 是在 str_data 里滑行的下标
        {
            byte_last = byte_curr;
            byte_curr = str_data.at(i);
            if (byte_last == FRAME_HEADER1 && byte_curr == FRAME_HEADER2 && rec_state == STATE_WAIT_FOR_HEADER)
                {
                    rec_state       = STATE_READING_FRAME;
                    rec_buffer[j++] = byte_last;
                    rec_buffer[j++] = byte_curr;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "found frame head at %d", j);
                }
            else if (rec_state == STATE_READING_FRAME)
                {
                    rec_buffer[j++] = str_data.at(i);
                    if (byte_last == FRAME_FOOTER1 && byte_curr == FRAME_FOOTER2)
                        {
                            frame_num++;
                            rec_state = STATE_WAIT_FOR_HEADER;
                        }
                }
            else
                rec_state = STATE_WAIT_FOR_HEADER;
        }
    if (frame_num < 1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "frame body not found");
            return parse_result;
        }

    // get the upload data
    uint16_t frame_len  = 0;                      // 第 k 个帧的帧长度, 帧长度 = 数据长度 + 帧头帧尾
    uint16_t buffer_loc = 0;                      // 解析到第 buffer_loc 个字节
    for (size_t i = 0, k = 0; k < frame_num; k++) // 解析了 k 个帧， 共 frame_num 个帧
                                                  // 特定帧中, 解析到第 i 个字节, 也是在帧数据中滑行
        {
            frame_len        = FRAME_SIZE(rec_buffer[buffer_loc + FRAME_DATA_LEN_LOC]);
            bool valid_frame = validateFrame(rec_buffer + buffer_loc, frame_len);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "frame %d, length %d, is valid %d", k, frame_len,
                                 valid_frame);
            if (rec_buffer[0 + buffer_loc] == FRAME_HEADER1 && rec_buffer[1 + buffer_loc] == FRAME_HEADER2 &&
                rec_buffer[frame_len - 2 + buffer_loc] == FRAME_FOOTER1 && rec_buffer[frame_len - 1 + buffer_loc] == FRAME_FOOTER2 && valid_frame)
                {
                    i = 4 + buffer_loc;
                    if (CMD_TYPE_DIFF_MOTO_DATA == rec_buffer[3 + buffer_loc])
                        {
                            for (int j = 0; j < 4; j++) // j for 4 motors
                                {
                                    speed_rpm_.int32_dat                                = 0;
                                    total_angle_.int32_dat                              = 0;
                                    round_cnt_.int32_dat                                = 0;

                                    speed_rpm_.byte_data[3]                             = rec_buffer[i++];
                                    speed_rpm_.byte_data[2]                             = rec_buffer[i++];
                                    speed_rpm_.byte_data[1]                             = rec_buffer[i++];
                                    speed_rpm_.byte_data[0]                             = rec_buffer[i++];
                                    chassis_measure_ptr->moto_measurements[j].speed_rpm = speed_rpm_.int32_dat; //*1000

                                    round_cnt_.byte_data[3]                             = rec_buffer[i++];
                                    round_cnt_.byte_data[2]                             = rec_buffer[i++];
                                    round_cnt_.byte_data[1]                             = rec_buffer[i++];
                                    round_cnt_.byte_data[0]                             = rec_buffer[i++];
                                    chassis_measure_ptr->moto_measurements[j].round_cnt = round_cnt_.int32_dat;

                                    total_angle_.byte_data[3]                           = rec_buffer[i++];
                                    total_angle_.byte_data[2]                           = rec_buffer[i++];
                                    total_angle_.byte_data[1]                           = rec_buffer[i++];
                                    total_angle_.byte_data[0]                           = rec_buffer[i++];
                                    chassis_measure_ptr->moto_measurements[j].angle     = total_angle_.int32_dat;

                                    chassis_measure_ptr->moto_measurements[j].available = 0x01;

                                    parse_result                                        = true;
                                }
                        }
                    else if (CMD_TYPE_RC_DATA == rec_buffer[3 + buffer_loc])
                        {
                            chassis_measure_ptr->rc.ch1 = rec_buffer[i++];
                            chassis_measure_ptr->rc.ch1 = (chassis_measure_ptr->rc.ch1 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.ch2 = rec_buffer[i++];
                            chassis_measure_ptr->rc.ch2 = (chassis_measure_ptr->rc.ch2 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.ch3 = rec_buffer[i++];
                            chassis_measure_ptr->rc.ch3 = (chassis_measure_ptr->rc.ch3 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.ch4 = rec_buffer[i++];
                            chassis_measure_ptr->rc.ch4 = (chassis_measure_ptr->rc.ch4 << 8) + rec_buffer[i++];

                            chassis_measure_ptr->rc.sw1 = rec_buffer[i++];
                            chassis_measure_ptr->rc.sw1 = (chassis_measure_ptr->rc.sw1 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.sw2 = rec_buffer[i++];
                            chassis_measure_ptr->rc.sw2 = (chassis_measure_ptr->rc.sw2 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.sw3 = rec_buffer[i++];
                            chassis_measure_ptr->rc.sw3 = (chassis_measure_ptr->rc.sw3 << 8) + rec_buffer[i++];
                            chassis_measure_ptr->rc.sw4 = rec_buffer[i++];
                            chassis_measure_ptr->rc.sw4 = (chassis_measure_ptr->rc.sw4 << 8) + rec_buffer[i++];

                            parse_result                = true;
                        }
                    else if (CMD_TYPE_ODOM_VEL == rec_buffer[3 + buffer_loc])
                        {
                            odom_.int16_dat                                  = 0;
                            odom_.int16_dat                                  = 0;
                            odom_.byte_data[1]                               = rec_buffer[i++];
                            odom_.byte_data[0]                               = rec_buffer[i++];
                            chassis_measure_ptr->odom_measurements.vx        = odom_.int16_dat / 1000.0f;

                            odom_.int16_dat                                  = 0;
                            odom_.int16_dat                                  = 0;
                            odom_.byte_data[1]                               = rec_buffer[i++];
                            odom_.byte_data[0]                               = rec_buffer[i++];
                            chassis_measure_ptr->odom_measurements.vy        = odom_.int16_dat / 1000.0f;

                            odom_.int16_dat                                  = 0;
                            odom_.int16_dat                                  = 0;
                            odom_.byte_data[1]                               = rec_buffer[i++];
                            odom_.byte_data[0]                               = rec_buffer[i++];
                            chassis_measure_ptr->odom_measurements.wz        = odom_.int16_dat / 1000.0f;

                            chassis_measure_ptr->odom_measurements.available = 0x01;

                            parse_result                                     = true;
                        }
                    else if (CMD_TYPE_GPIO == rec_buffer[3 + buffer_loc])
                        {
                            ; // do nothing
                        }
                    else
                        {
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "unrecognize frame type %#X",
                                                 (int)rec_buffer[3 + buffer_loc]);
                            parse_result = false;
                        }
                }
            buffer_loc += frame_len;
        }
    return parse_result;
}

bool MickRobotVehicleInterface::validateFrame(uint8_t *frame, uint16_t frameLength)
{
    for (size_t i = 0; i < frameLength; i++)
        {
            RCLCPP_INFO_STREAM(get_logger(), "frame data " << std::showbase << std::hex << std::uppercase << (int)frame[i]);
        }

    // 数据长度位 (第三个字节)
    uint8_t dataLength         = frame[2];
    // 校验位(倒数第三个字节)
    uint8_t checksum           = frame[frameLength - 3];
    // 除开帧头帧尾校验位的所有数据加和
    // 计算校验位
    uint8_t calculatedChecksum = 0;
    // 将数据部分的每个字节（低8位）加到校验位中
    for (uint16_t i = 2; i < 2 + dataLength - 1; i++)
        {
            calculatedChecksum += frame[i] & 0xFF; // 取低8位
        }
    // 校验相等
    return (calculatedChecksum == checksum);
}