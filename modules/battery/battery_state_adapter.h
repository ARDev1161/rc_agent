#ifndef BATTERY_STATE_ADAPTER_H_
#define BATTERY_STATE_ADAPTER_H_

#include <sensor_msgs/msg/battery_state.hpp>

namespace BatteryAdapter {

/**
 * @brief Generic adapter that maps ROS BatteryState to protobuf BatteryState-like messages.
 *
 * The target protobuf type is expected to expose setters compatible with the generated code
 * (e.g. set_voltage, set_power_supply_status, add_cell_voltage, etc.) and nested enum types
 * PowerSupplyStatus, PowerSupplyHealth, PowerSupplyTechnology.
 */
template <typename ProtoBatteryState>
ProtoBatteryState batteryStateToProtoCommon(const sensor_msgs::msg::BatteryState &ros_msg);

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t ros_status);

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t ros_health);

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t ros_tech);

// ---- Template implementations ------------------------------------------------

template <typename ProtoBatteryState>
ProtoBatteryState batteryStateToProtoCommon(const sensor_msgs::msg::BatteryState &ros_msg)
{
  ProtoBatteryState proto_battery_state;

  proto_battery_state.set_voltage(ros_msg.voltage);
  proto_battery_state.set_temperature(ros_msg.temperature);
  proto_battery_state.set_current(ros_msg.current);
  proto_battery_state.set_charge(ros_msg.charge);
  proto_battery_state.set_capacity(ros_msg.capacity);
  proto_battery_state.set_design_capacity(ros_msg.design_capacity);
  proto_battery_state.set_percentage(ros_msg.percentage);

  proto_battery_state.set_power_supply_status(
      convertPowerSupplyStatus<ProtoBatteryState>(ros_msg.power_supply_status));
  proto_battery_state.set_power_supply_health(
      convertPowerSupplyHealth<ProtoBatteryState>(ros_msg.power_supply_health));
  proto_battery_state.set_power_supply_technology(
      convertPowerSupplyTechnology<ProtoBatteryState>(ros_msg.power_supply_technology));

  proto_battery_state.set_present(ros_msg.present);

  for (auto &voltage : ros_msg.cell_voltage) {
    proto_battery_state.add_cell_voltage(voltage);
  }

  for (auto &temp : ros_msg.cell_temperature) {
    proto_battery_state.add_cell_temperature(temp);
  }

  proto_battery_state.set_location(ros_msg.location);
  proto_battery_state.set_serial_number(ros_msg.serial_number);

  return proto_battery_state;
}

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t batteryStatus)
{
  switch (batteryStatus) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      return ProtoBatteryState::CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      return ProtoBatteryState::DISCHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return ProtoBatteryState::NOT_CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      return ProtoBatteryState::FULL;
    default:
      return ProtoBatteryState::UNKNOWN;
  }
}

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t batteryHealth)
{
  switch (batteryHealth) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD:
      return ProtoBatteryState::HEALTH_GOOD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:
      return ProtoBatteryState::OVERHEAT;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD:
      return ProtoBatteryState::DEAD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:
      return ProtoBatteryState::OVERVOLTAGE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
      return ProtoBatteryState::UNSPEC_FAILURE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD:
      return ProtoBatteryState::COLD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
      return ProtoBatteryState::WATCHDOG_TIMER_EXPIRE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
      return ProtoBatteryState::SAFETY_TIMER_EXPIRE;
    default:
      return ProtoBatteryState::HEALTH_UNKNOWN;
  }
}

template <typename ProtoBatteryState>
typename ProtoBatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t batteryTech)
{
  switch (batteryTech) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH:
      return ProtoBatteryState::NIMH;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION:
      return ProtoBatteryState::LION;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO:
      return ProtoBatteryState::LIPO;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE:
      return ProtoBatteryState::LIFE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD:
      return ProtoBatteryState::NICD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN:
      return ProtoBatteryState::LIMN;
    default:
      return ProtoBatteryState::TECHNOLOGY_UNKNOWN;
  }
}

}  // namespace BatteryAdapter

#endif  // BATTERY_STATE_ADAPTER_H_
