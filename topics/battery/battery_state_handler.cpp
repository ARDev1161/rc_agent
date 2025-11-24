#include "battery_state_handler.h"

Robot::Sensors::BatteryState batteryStateToProto(const sensor_msgs::msg::BatteryState &ros_msg)
{
  Robot::Sensors::BatteryState proto_battery_state;

  // Заполняем поля
  proto_battery_state.set_voltage(ros_msg.voltage);
  proto_battery_state.set_temperature(ros_msg.temperature);
  proto_battery_state.set_current(ros_msg.current);
  proto_battery_state.set_charge(ros_msg.charge);
  proto_battery_state.set_capacity(ros_msg.capacity);
  proto_battery_state.set_design_capacity(ros_msg.design_capacity);
  proto_battery_state.set_percentage(ros_msg.percentage);

  proto_battery_state.set_power_supply_status(
      convertPowerSupplyStatus(ros_msg.power_supply_status));
  proto_battery_state.set_power_supply_health(
      convertPowerSupplyHealth(ros_msg.power_supply_health));
  proto_battery_state.set_power_supply_technology(
      convertPowerSupplyTechnology(ros_msg.power_supply_technology));

  proto_battery_state.set_present(ros_msg.present);

  // Копируем cell_voltage
  for (auto &voltage : ros_msg.cell_voltage) {
    proto_battery_state.add_cell_voltage(voltage);
  }

  // Копируем cell_temperature
  for (auto &temp : ros_msg.cell_temperature) {
    proto_battery_state.add_cell_temperature(temp);
  }

  proto_battery_state.set_location(ros_msg.location);
  proto_battery_state.set_serial_number(ros_msg.serial_number);

  return proto_battery_state;
}

Robot::Sensors::BatteryState::PowerSupplyStatus convertPowerSupplyStatus(uint8_t batteryStatus)
{
  switch (batteryStatus) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      return Robot::Sensors::BatteryState::CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      return Robot::Sensors::BatteryState::DISCHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return Robot::Sensors::BatteryState::NOT_CHARGING;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      return Robot::Sensors::BatteryState::FULL;
    default:
      return Robot::Sensors::BatteryState::UNKNOWN;
  }
}

Robot::Sensors::BatteryState::PowerSupplyHealth convertPowerSupplyHealth(uint8_t batteryHealth)
{
  switch (batteryHealth) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD:
      return Robot::Sensors::BatteryState::HEALTH_GOOD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:
      return Robot::Sensors::BatteryState::OVERHEAT;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD:
      return Robot::Sensors::BatteryState::DEAD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:
      return Robot::Sensors::BatteryState::OVERVOLTAGE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
      return Robot::Sensors::BatteryState::UNSPEC_FAILURE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD:
      return Robot::Sensors::BatteryState::COLD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
      return Robot::Sensors::BatteryState::WATCHDOG_TIMER_EXPIRE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
      return Robot::Sensors::BatteryState::SAFETY_TIMER_EXPIRE;
    default:
      return Robot::Sensors::BatteryState::HEALTH_UNKNOWN;
  }
}

Robot::Sensors::BatteryState::PowerSupplyTechnology convertPowerSupplyTechnology(uint8_t batteryTech)
{
  switch (batteryTech) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH:
      return Robot::Sensors::BatteryState::NIMH;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION:
      return Robot::Sensors::BatteryState::LION;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO:
      return Robot::Sensors::BatteryState::LIPO;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE:
      return Robot::Sensors::BatteryState::LIFE;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD:
      return Robot::Sensors::BatteryState::NICD;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN:
      return Robot::Sensors::BatteryState::LIMN;
    default:
      return Robot::Sensors::BatteryState::TECHNOLOGY_UNKNOWN;
  }
}
