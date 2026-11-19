#ifndef BATTERY_STATE_NODE_H_
#define BATTERY_STATE_NODE_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "battery_state_handler.h"

/**
 * @brief Node class for handling battery state messages.
 *
 * This class inherits from rclcpp::Node and subscribes to a topic
 * publishing sensor_msgs::msg::BatteryState messages. It converts the received
 * messages to a protobuf format using batteryStateToProto() and publishes the latest reading into the shared gRPC Sensors snapshot.
 */
class BatteryStateNode : public rclcpp::Node
{
  /// Pointer to a BatteryState object for storing or further processing data.
  std::shared_ptr<Robot::Sensors> sensors_;
  std::shared_ptr<std::mutex> sensors_mutex_;
  std::chrono::steady_clock::time_point last_message_{};
  bool received_message_{false};
  rclcpp::TimerBase::SharedPtr stale_timer_;
  std::chrono::milliseconds stale_timeout_;
  void clearStaleBattery();
public:
  /**
   * @brief Constructor for BatteryStateNode.
   *
   * Initializes the node with the name "battery_state_node", stores the pointer to
   * the BatteryState object, and subscribes to the specified topic.
   *
   * @param sensors Shared outgoing gRPC Sensors snapshot.
   * @param sensors_mutex Mutex shared with the gRPC sender.
   * @param topicName ROS2 battery topic to subscribe to.
   * @param stale_timeout Maximum time without messages before clearing the field.
   */
  BatteryStateNode(std::shared_ptr<Robot::Sensors> sensors,
                   std::shared_ptr<std::mutex> sensors_mutex,
                   const std::string &topicName,
                   std::chrono::milliseconds stale_timeout = std::chrono::seconds(5));

private:
  /**
   * @brief Callback function for processing battery state messages.
   *
   * This function is called whenever a new message is received on the subscribed topic.
   * It converts the ROS message to a protobuf object using batteryStateToProto(),
   * updates the shared outgoing Sensors snapshot.
   *
   * @param msg Shared pointer to a sensor_msgs::msg::BatteryState message.
   */
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  /// Subscriber for sensor_msgs::msg::BatteryState messages.
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};

#endif  // BATTERY_STATE_NODE_H_
