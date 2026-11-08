#include "battery_state_node.h"

BatteryStateNode::BatteryStateNode(std::shared_ptr<Robot::Sensors> sensors,
                                   std::shared_ptr<std::mutex> sensors_mutex,
                                   const std::string &topicName)
: Node("battery_state_node"),
  sensors_(std::move(sensors)),
  sensors_mutex_(std::move(sensors_mutex))
{
  subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
      topicName, rclcpp::SensorDataQoS(),
      std::bind(&BatteryStateNode::batteryStateCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Optional BatteryState subscription: %s", topicName.c_str());
}

void BatteryStateNode::batteryStateCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (!msg || !sensors_ || !sensors_mutex_) {
    return;
  }

  // Convert outside the lock; DataExchange and DataStreamExchange snapshot
  // Sensors under this exact same mutex.
  auto battery = batteryStateToProto(*msg);
  std::lock_guard<std::mutex> lock(*sensors_mutex_);
  *sensors_->mutable_batterystate() = std::move(battery);
}
