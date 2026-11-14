#include "stream_pipeline_receiver.h"

#include <utility>

StreamPipelineReceiver::StreamPipelineReceiver(
    std::shared_ptr<Robot::Sensors> sensors,
    std::shared_ptr<std::mutex> sensors_mutex)
: Node("stream_pipeline_receiver"),
  sensors_(std::move(sensors)),
  sensors_mutex_(std::move(sensors_mutex))
{
  pipeline_topic_ = declare_parameter<std::string>("pipeline_topic", "/gst/pipeline");
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  sub_ = create_subscription<std_msgs::msg::String>(
      pipeline_topic_, qos,
      std::bind(&StreamPipelineReceiver::onPipeline, this, std::placeholders::_1));
}

void StreamPipelineReceiver::onPipeline(
    const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || !sensors_ || !sensors_mutex_ || msg->data.empty()) {
    return;
  }

  // DataExchange()/DataStreamExchange() take this exact mutex while copying
  // the Sensors snapshot. Keep the mutation and duplicate check together.
  {
    std::lock_guard<std::mutex> lock(*sensors_mutex_);
    if (msg->data == last_pipeline_) {
      return;
    }
    last_pipeline_ = msg->data;
    sensors_->mutable_video_stream()->set_pipeline(last_pipeline_);
  }

  RCLCPP_INFO(get_logger(), "Updated video pipeline from %s",
              pipeline_topic_.c_str());
}
