#include "battery_state_node.h"

BatteryStateNode::BatteryStateNode(BatteryState* batteryStatePtr, std::string topicName)
: Node("battery_state_node"),
  batteryStatePtr(batteryStatePtr)
{
  // Create a subscription for sensor_msgs::msg::BatteryState messages
  subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      topicName,
      10,
      std::bind(&BatteryStateNode::batteryStateCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "BatteryStateNode initialized");
}

void BatteryStateNode::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  // Convert the ROS message to a protobuf object
  auto proto_battery_state = batteryStateToProto(*msg);

  // Log battery state information
  RCLCPP_INFO(
    this->get_logger(),
    "Battery callback: voltage=%.2f, percentage=%.2f",
    msg->voltage,
    msg->percentage
  );

  // Print the debug string of the protobuf object
  std::cout << proto_battery_state.DebugString() << std::endl;
}
