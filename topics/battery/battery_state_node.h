#ifndef BATTERY_STATE_NODE_H_
#define BATTERY_STATE_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class BatteryStateNode : public rclcpp::Node
{
public:
  BatteryStateNode();

private:
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};

#endif  // BATTERY_STATE_NODE_H_
