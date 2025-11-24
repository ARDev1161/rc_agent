#ifndef BATTERY_STATE_HANDLER_H_
#define BATTERY_STATE_HANDLER_H_

#include <sensor_msgs/msg/battery_state.hpp>

#include "robot.grpc.pb.h"

using BatteryState = Power::BatteryState;

/**
 * @brief Converts a ROS BatteryState message to a BatteryState protobuf object.
 *
 * This function converts the sensor_msgs::msg::BatteryState ROS message into a
 * BatteryState protobuf object, mapping all relevant fields.
 *
 * @param ros_msg A reference to the ROS BatteryState message.
 * @return BatteryState A protobuf BatteryState object with the converted data.
 */
BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg);

/**
 * @brief Converts ROS battery power supply status to a BatteryState enumeration.
 *
 * This function maps the ROS battery power supply status (provided as a uint8_t)
 * to the corresponding BatteryState::PowerSupplyStatus enumeration value.
 *
 * @param ros_status The ROS battery power supply status value.
 * @return BatteryState::PowerSupplyStatus The mapped power supply status.
 */
BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t ros_status);

/**
 * @brief Converts ROS battery power supply health to a BatteryState enumeration.
 *
 * This function maps the ROS battery power supply health (provided as a uint8_t)
 * to the corresponding BatteryState::PowerSupplyHealth enumeration value.
 *
 * @param ros_health The ROS battery power supply health value.
 * @return BatteryState::PowerSupplyHealth The mapped power supply health.
 */
BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t ros_health);

/**
 * @brief Converts ROS battery power supply technology to a BatteryState enumeration.
 *
 * This function maps the ROS battery power supply technology (provided as a uint8_t)
 * to the corresponding BatteryState::PowerSupplyTechnology enumeration value.
 *
 * @param ros_tech The ROS battery power supply technology value.
 * @return BatteryState::PowerSupplyTechnology The mapped power supply technology.
 */
BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t ros_tech);

#endif  // BATTERY_STATE_HANDLER_H_
