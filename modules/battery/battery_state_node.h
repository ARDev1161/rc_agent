#ifndef BATTERY_STATE_NODE_H_
#define BATTERY_STATE_NODE_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "battery_state_handler.h"

/**
 * @brief Node class for handling battery state messages.
 *
 * This class inherits from rclcpp::Node and subscribes to a topic
 * publishing sensor_msgs::msg::BatteryState messages. It converts the received
 * messages to a protobuf format using batteryStateToProto() and logs the information.
 */
class BatteryStateNode : public rclcpp::Node
{
  /// Pointer to a BatteryState object for storing or further processing data.
  BatteryState* batteryStatePtr;
public:
  /**
   * @brief Constructor for BatteryStateNode.
   *
   * Initializes the node with the name "battery_state_node", stores the pointer to
   * the BatteryState object, and subscribes to the specified topic.
   *
   * @param batteryStatePtr Pointer to the BatteryState object for data storage.
   * @param topicName Name of the topic to subscribe to.
   */
  BatteryStateNode(BatteryState* batteryStatePtr, std::string topicName);

private:
  /**
   * @brief Callback function for processing battery state messages.
   *
   * This function is called whenever a new message is received on the subscribed topic.
   * It converts the ROS message to a protobuf object using batteryStateToProto(),
   * logs the basic battery parameters, and prints the protobuf debug string.
   *
   * @param msg Shared pointer to a sensor_msgs::msg::BatteryState message.
   */
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  /// Subscriber for sensor_msgs::msg::BatteryState messages.
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};

#endif  // BATTERY_STATE_NODE_H_
