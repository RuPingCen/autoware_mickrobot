# autoware_mickrobot
该节点是为适配autoware.universe自动驾驶框架而编写的ROS2节点，目前仅支持 [mick_robot_chassis](https://github.com/RuPingCen/mick_robot_chassis) 开源底盘。

适配工作参考了Autoware教程 [Creating vehicle interface](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/creating-vehicle-interface/)


该节点目前实现了以下话题收发：

订阅：

| Topic Name                           | Topic Type                                             | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------------------------ | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| /control/command/control_cmd         | autoware_auto_control_msgs/msg/AckermannControlCommand | This topic includes main topics for controlling our vehicle like a steering tire angle, speed, acceleration, etc.                                                                                                                                                                                                                                                                                                                                                                                                               |
| /control/command/gear_cmd            | autoware_auto_vehicle_msgs/msg/GearCommand             | This topic includes gear command for autonomous driving, please check message values to make sense of gears values. Please check [the message definition](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearCommand.idl) of this type.                                                                                                                                                                                                                                             |
| /control/command/emergency_cmd       | tier4_vehicle_msgs/msg/VehicleEmergencyStamped         | This topic sends emergency when autoware is on emergency state. Please check [VehicleEmergencyStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/VehicleEmergencyStamped.msg) message type for detailed information.                                                                                                                                                                                                                                                              |
| /control/command/turn_indicators_cmd | autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand   | This topic indicates a turn signal for your own vehicle. Please check [TurnIndicatorsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl) message type for detailed information.                                                                                                                                                                                                                                                                      |
| /control/command/hazard_lights_cmd   | autoware_auto_vehicle_msgs/msg/HazardLightsCommand     | This topic sends command for hazard lights. Please check [HazardLightsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsCommand.idl)                                                                                                                                                                                                                                                                                                                              |




 发布：
 | Topic Name                             | Topic Type                                          | Description                                                                                                                                                                                                                                                                   |
| -------------------------------------- | --------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    | This topic describes the current control mode of vehicle. Please check [ControlModeReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/ControlModeReport.idl) message type for detailed information.                           |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           | This topic includes the current gear status of the vehicle. Please check [GearReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearReport.idl) message type for detailed information.                                       |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       | This topic gives us the velocity status of the vehicle. Please check [VelocityReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/VelocityReport.idl) message type for detailed information.                                   |
| /vehicle/status/hazard_lights_status   | autoware_auto_vehicle_msgs/msg/HazardLightsReport   | This topic describes hazard light status of the vehicle. Please check [HazardLightsReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsReport.idl) message type for detailed information.                          |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport | This topic reports the turn indicators status of the vehicle. Please check [TurnIndicatorsReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport.idl) message type for detailed information.                 |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/msg/SteeringReport       | This topic reports the steering status of the vehicle. Please check [SteeringReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/SteeringReport.idl) message type for detailed information.                                    |


经过实际考察验证，至少需要收发以下话题。此外，针对 [mick_robot_chassis](https://github.com/RuPingCen/mick_robot_chassis) 底盘，部分部件 (如转向灯) 实际不存在, 采用在 interface 内部维护一个变量方式提供。  

至少需要接受以下话题    
 - 转向、速度、加速度
    /control/command/control_cmd
 - 紧急停车
    /control/command/emergency_cmd
 - 档位控制
    /control/command/gear_cmd
 - 转向灯
    /control/command/turn_indicators_cmd
 - 双闪 危险报警闪光灯
    /control/command/hazard_lights_cmd

至少需要发布以下话题    
 - 车辆档位状态
    /vehicle/status/gear_status
 - 上报车辆控制模式 用于确定是否被人工接管
    /vehicle/status/control_mode
 - 上报车速
    /vehicle/status/velocity_status
 - 上报转向
    /vehicle/status/steering_status
 - 上报转向灯
    /vehicle/status/turn_indicators_status
 - 上报双闪灯
    /vehicle/status/hazard_lights_status