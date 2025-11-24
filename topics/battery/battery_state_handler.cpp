#include "battery_state_handler.h"
#include "../../modules/battery/battery_state_adapter.h"

Robot::Sensors::BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg)
{
  return BatteryAdapter::batteryStateToProtoCommon<Robot::Sensors::BatteryState>(ros_msg);
}

Robot::Sensors::BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t batteryStatus)
{
  return BatteryAdapter::convertPowerSupplyStatus<Robot::Sensors::BatteryState>(batteryStatus);
}

Robot::Sensors::BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t batteryHealth)
{
  return BatteryAdapter::convertPowerSupplyHealth<Robot::Sensors::BatteryState>(batteryHealth);
}

Robot::Sensors::BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t batteryTech)
{
  return BatteryAdapter::convertPowerSupplyTechnology<Robot::Sensors::BatteryState>(batteryTech);
}
