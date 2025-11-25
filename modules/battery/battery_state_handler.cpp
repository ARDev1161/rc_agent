#include "battery_state_handler.h"
#include "battery_state_adapter.h"

BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg)
{
  return BatteryAdapter::batteryStateToProtoCommon<BatteryState>(ros_msg);
}

BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t batteryStatus)
{
  return BatteryAdapter::convertPowerSupplyStatus<BatteryState>(batteryStatus);
}

BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t batteryHealth)
{
  return BatteryAdapter::convertPowerSupplyHealth<BatteryState>(batteryHealth);
}

BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t batteryTech)
{
  return BatteryAdapter::convertPowerSupplyTechnology<BatteryState>(batteryTech);
}
