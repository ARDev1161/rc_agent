#include "battery_state_node.h"

BatteryStateNode::BatteryStateNode(std::shared_ptr<Robot::Sensors> sensors,
                                   std::shared_ptr<std::mutex> sensors_mutex,
                                   const std::string &topicName,
                                   std::chrono::milliseconds stale_timeout)
: Node("battery_state_node"),
  sensors_(std::move(sensors)),
  sensors_mutex_(std::move(sensors_mutex)),
  stale_timeout_(stale_timeout)
{
  subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
      topicName, rclcpp::SensorDataQoS(),
      std::bind(&BatteryStateNode::batteryStateCallback, this, std::placeholders::_1));
  stale_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&BatteryStateNode::clearStaleBattery, this));
  RCLCPP_INFO(get_logger(), "Optional BatteryState subscription: %s", topicName.c_str());
}

void BatteryStateNode::batteryStateCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (!msg || !sensors_ || !sensors_mutex_) {
    return;
  }

  // Convert outside the lock; the gRPC transport snapshots Sensors under
  // this same mutex. Guard the freshness timestamp as well, including when
  // a multi-threaded executor uses separate callback groups in the future.
  auto battery = batteryStateToProto(*msg);
  std::lock_guard<std::mutex> lock(*sensors_mutex_);
  // A ROS publisher can explicitly report that its pack is absent. Do not
  // keep an optional protobuf battery in that case, or display the previous
  // percentage until the timeout expires.
  if (!msg->present) {
    sensors_->clear_batterystate();
    received_message_ = false;
    return;
  }
  *sensors_->mutable_batterystate() = std::move(battery);
  last_message_ = std::chrono::steady_clock::now();
  received_message_ = true;
}

void BatteryStateNode::clearStaleBattery()
{
  if (!sensors_ || !sensors_mutex_) {
    return;
  }
  std::lock_guard<std::mutex> lock(*sensors_mutex_);
  if (!received_message_ ||
      std::chrono::steady_clock::now() - last_message_ < stale_timeout_) {
    return;
  }
  sensors_->clear_batterystate();
  received_message_ = false;
  RCLCPP_WARN(get_logger(), "BatteryState timed out; clearing stale telemetry");
}
