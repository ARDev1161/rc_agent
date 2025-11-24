#include "battery_state_handler.h"

/// @cond DOXYGEN_SHOULD_SKIP

BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg)
{
  BatteryState proto_battery_state;

  // Map basic battery parameters
  proto_battery_state.set_voltage(ros_msg.voltage);
  proto_battery_state.set_temperature(ros_msg.temperature);
  proto_battery_state.set_current(ros_msg.current);
  proto_battery_state.set_charge(ros_msg.charge);
  proto_battery_state.set_capacity(ros_msg.capacity);
  proto_battery_state.set_design_capacity(ros_msg.design_capacity);
  proto_battery_state.set_percentage(ros_msg.percentage);

  // Map power supply status, health, and technology
  proto_battery_state.set_power_supply_status(
      convertPowerSupplyStatus(ros_msg.power_supply_status));
  proto_battery_state.set_power_supply_health(
      convertPowerSupplyHealth(ros_msg.power_supply_health));
  proto_battery_state.set_power_supply_technology(
      convertPowerSupplyTechnology(ros_msg.power_supply_technology));

  proto_battery_state.set_present(ros_msg.present);

  // Copy cell voltage values
  for (auto &voltage : ros_msg.cell_voltage) {
    proto_battery_state.add_cell_voltage(voltage);
  }

  // Copy cell temperature values
  for (auto &temp : ros_msg.cell_temperature) {
    proto_battery_state.add_cell_temperature(temp);
  }

  proto_battery_state.set_location(ros_msg.location);
  proto_battery_state.set_serial_number(ros_msg.serial_number);

  return proto_battery_state;
}

BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t batteryStatus)
{
  switch (batteryStatus) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      return BatteryState::CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      return BatteryState::DISCHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return BatteryState::NOT_CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      return BatteryState::FULL;
    default:
      return BatteryState::UNKNOWN;
  }
}

BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t batteryHealth)
{
  switch (batteryHealth) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD:
      return BatteryState::HEALTH_GOOD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:
      return BatteryState::OVERHEAT;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD:
      return BatteryState::DEAD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:
      return BatteryState::OVERVOLTAGE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
      return BatteryState::UNSPEC_FAILURE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD:
      return BatteryState::COLD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
      return BatteryState::WATCHDOG_TIMER_EXPIRE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
      return BatteryState::SAFETY_TIMER_EXPIRE;
    default:
      return BatteryState::HEALTH_UNKNOWN;
  }
}

BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t batteryTech)
{
  switch (batteryTech) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH:
      return BatteryState::NIMH;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION:
      return BatteryState::LION;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO:
      return BatteryState::LIPO;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE:
      return BatteryState::LIFE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD:
      return BatteryState::NICD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN:
      return BatteryState::LIMN;
    default:
      return BatteryState::TECHNOLOGY_UNKNOWN;
  }
}

/// @endcond
