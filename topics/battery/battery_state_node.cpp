#include "battery_state_node.h"
#include "battery_state_handler.h"

BatteryStateNode::BatteryStateNode()
: Node("battery_state_node")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state",
      10,
      std::bind(&BatteryStateNode::batteryStateCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "BatteryStateNode has been started.");
}

void BatteryStateNode::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  auto proto_battery_state = batteryStateToProto(*msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Battery callback: voltage=%.2f, percentage=%.2f",
    msg->voltage,
    msg->percentage
  );
}
