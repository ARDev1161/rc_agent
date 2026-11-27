// Real ROS2 publisher/subscriber regression for legacy video pipeline metadata.
// No GStreamer process, camera hardware or physical robot is required.
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "modules/video/stream_pipeline_receiver.h"

using namespace std::chrono_literals;

TEST(VideoPipelineSnapshot, RosCallbackUsesGrpcSensorsMutex)
{
  int argc = 0;
  rclcpp::init(argc, nullptr);
  struct ShutdownGuard {
    ~ShutdownGuard() { if (rclcpp::ok()) rclcpp::shutdown(); }
  } shutdown_guard;

  auto sensors = std::make_shared<Robot::Sensors>();
  sensors->mutable_robot_info()->set_robot_id("video-test-robot");
  sensors->mutable_batterystate()->set_percentage(0.42f);
  auto grpc_mutex = std::make_shared<std::mutex>();

  auto receiver = std::make_shared<StreamPipelineReceiver>(sensors, grpc_mutex);
  auto publisher_node = std::make_shared<rclcpp::Node>("video_pipeline_test_publisher");
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
      "/gst/pipeline", qos);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(receiver);
  executor.add_node(publisher_node);

  auto publish_until = [&](const std::string &pipeline) {
    std_msgs::msg::String msg;
    msg.data = pipeline;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      publisher->publish(msg);
      executor.spin_some();
      {
        std::lock_guard<std::mutex> lock(*grpc_mutex);
        if (sensors->video_stream().pipeline() == pipeline) {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  };

  // No ROS publication must not fabricate an arbitrary GStreamer pipeline.
  for (int i = 0; i < 20; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  {
    std::lock_guard<std::mutex> lock(*grpc_mutex);
    ASSERT_TRUE(sensors->video_stream().pipeline().empty());
  }

  ASSERT_TRUE(publish_until("videotestsrc ! fakesink"));
  // Even a very long pipeline must remain a coherent string in the copy
  // visible to the grpcClient sender (no unprotected in-place mutation).
  const std::string long_pipeline =
      "videotestsrc ! videoconvert ! " + std::string(16384, 'x') + " ! fakesink";
  ASSERT_TRUE(publish_until(long_pipeline));

  // Empty updates are ignored; neither the video pipeline nor robot
  // descriptor/battery should be erased by this optional ROS topic.
  std_msgs::msg::String empty;
  for (int i = 0; i < 10; ++i) {
    publisher->publish(empty);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  {
    std::lock_guard<std::mutex> lock(*grpc_mutex);
    EXPECT_EQ(sensors->video_stream().pipeline(), long_pipeline);
    EXPECT_EQ(sensors->robot_info().robot_id(), "video-test-robot");
    EXPECT_FLOAT_EQ(sensors->batterystate().percentage(), 0.42f);
  }

  // The latest ROS message must not overwrite a gRPC snapshot while a
  // sender holds the same mutex. The executor thread blocks on the lock,
  // and delivery is then verified after release (no timing-only pass).
  std_msgs::msg::String next;
  next.data = "videotestsrc pattern=ball ! fakesink";
  std::thread spin_thread;
  {
    std::lock_guard<std::mutex> lock(*grpc_mutex);
    publisher->publish(next);
    spin_thread = std::thread([&executor] { executor.spin_some(); });
    std::this_thread::sleep_for(75ms);
    EXPECT_EQ(sensors->video_stream().pipeline(), long_pipeline);
  }
  spin_thread.join();
  ASSERT_TRUE(publish_until(next.data));
  {
    std::lock_guard<std::mutex> lock(*grpc_mutex);
    EXPECT_EQ(sensors->robot_info().robot_id(), "video-test-robot");
    EXPECT_FLOAT_EQ(sensors->batterystate().percentage(), 0.42f);
  }
}
