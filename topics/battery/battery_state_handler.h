#ifndef BATTERY_STATE_HANDLER_H_
#define BATTERY_STATE_HANDLER_H_

#include <sensor_msgs/msg/battery_state.hpp>

#include "network/protobuf/robot.grpc.pb.h"

Robot::Sensors::BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg);

Robot::Sensors::BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t ros_status);
Robot::Sensors::BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t ros_health);
Robot::Sensors::BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t ros_tech);

#endif  // BATTERY_STATE_HANDLER_H_
