#ifndef RC_AGENT_STREAM_PIPELINE_RECEIVER_H_
#define RC_AGENT_STREAM_PIPELINE_RECEIVER_H_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robot.grpc.pb.h"

/**
 * @brief Receives the legacy GStreamer pipeline announcement from ROS2.
 *
 * Writes into the same Sensors protobuf used by the rc_agent gRPC client.
 * The caller must pass grpcClient::getMutex(), not a separate mutex, so
 * no ROS2 callback can mutate a protobuf during an outgoing RPC snapshot.
 * This does not control/start/stop GStreamer or select active robot video.
 */
class StreamPipelineReceiver : public rclcpp::Node {
public:
  StreamPipelineReceiver(std::shared_ptr<Robot::Sensors> sensors,
                         std::shared_ptr<std::mutex> sensors_mutex);

private:
  void onPipeline(const std_msgs::msg::String::SharedPtr msg);

  std::shared_ptr<Robot::Sensors> sensors_;
  std::shared_ptr<std::mutex> sensors_mutex_;
  std::string pipeline_topic_;
  std::string last_pipeline_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

#endif  // RC_AGENT_STREAM_PIPELINE_RECEIVER_H_
