#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "modules/battery/battery_state_node.h"

using namespace std::chrono_literals;

TEST(BatteryFreshness, PublishExpireAndRecover)
{
  int argc = 0;
  rclcpp::init(argc, nullptr);
  struct ShutdownGuard {
    ~ShutdownGuard() { if (rclcpp::ok()) rclcpp::shutdown(); }
  } shutdown_guard;
  auto sensors = std::make_shared<Robot::Sensors>();
  // Battery timeout must not erase unrelated robot identity/capabilities.
  sensors->mutable_robot_info()->set_robot_id("battery-test-robot");
  auto mutex = std::make_shared<std::mutex>();
  auto battery = std::make_shared<BatteryStateNode>(
      sensors, mutex, "/rc_api_v2_test_battery", 1200ms);
  auto publisher_node = std::make_shared<rclcpp::Node>("battery_test_publisher");
  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::BatteryState>(
      "/rc_api_v2_test_battery", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(battery);
  executor.add_node(publisher_node);

  // No publisher data must never fabricate a 0%-charged pack.
  for (int i = 0; i < 20; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  {
    std::lock_guard<std::mutex> lock(*mutex);
    ASSERT_FALSE(sensors->has_batterystate());
  }

  auto publish_and_wait = [&](float percentage) {
    sensor_msgs::msg::BatteryState message;
    message.present = true;
    message.percentage = percentage;
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      publisher->publish(message);
      executor.spin_some();
      {
        std::lock_guard<std::mutex> lock(*mutex);
        if (sensors->has_batterystate() &&
            sensors->batterystate().percentage() == percentage) {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  };

  ASSERT_TRUE(publish_and_wait(0.42f));
  {
    std::lock_guard<std::mutex> lock(*mutex);
    EXPECT_TRUE(sensors->has_batterystate());
  }

  // A second update while the publisher is active must extend freshness.
  // Use a 1200-ms timeout with 650/700-ms waits: the assertion crosses the
  // original deadline but leaves 500 ms of slack after the refresh. The
  // previous 350/200/300-ms timing was vulnerable to CI scheduler jitter.
  const auto first_sample_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - first_sample_time < 650ms) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(publish_and_wait(0.55f));
  const auto refreshed_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - refreshed_time < 700ms) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  {
    std::lock_guard<std::mutex> lock(*mutex);
    ASSERT_TRUE(sensors->has_batterystate());
    EXPECT_FLOAT_EQ(sensors->batterystate().percentage(), 0.55f);
  }

  // No further publications: the real ROS timer must clear the outgoing
  // snapshot. This checks the actual subscriber/timer, not a mock clock.
  auto deadline = std::chrono::steady_clock::now() + 3s;
  bool expired = false;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    {
      std::lock_guard<std::mutex> lock(*mutex);
      expired = !sensors->has_batterystate();
    }
    if (expired) break;
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(expired);
  {
    std::lock_guard<std::mutex> lock(*mutex);
    EXPECT_EQ(sensors->robot_info().robot_id(), "battery-test-robot");
  }
  ASSERT_TRUE(publish_and_wait(0.76f));
  {
    std::lock_guard<std::mutex> lock(*mutex);
    EXPECT_TRUE(sensors->has_batterystate());
    EXPECT_FLOAT_EQ(sensors->batterystate().percentage(), 0.76f);
  }

  // The next timeout must also clear the recovered value, not just the
  // initial sample. This catches stale received_message_ bookkeeping.
  auto second_deadline = std::chrono::steady_clock::now() + 3s;
  bool second_expired = false;
  while (std::chrono::steady_clock::now() < second_deadline) {
    executor.spin_some();
    {
      std::lock_guard<std::mutex> lock(*mutex);
      second_expired = !sensors->has_batterystate();
    }
    if (second_expired) break;
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(second_expired);
  {
    std::lock_guard<std::mutex> lock(*mutex);
    EXPECT_EQ(sensors->robot_info().robot_id(), "battery-test-robot");
  }
  ASSERT_TRUE(publish_and_wait(0.33f));
  {
    std::lock_guard<std::mutex> lock(*mutex);
    ASSERT_TRUE(sensors->has_batterystate());
    EXPECT_FLOAT_EQ(sensors->batterystate().percentage(), 0.33f);
  }

  // A publisher may explicitly remove its battery. Clear it immediately,
  // without waiting for the freshness timeout, and keep robot identity.
  {
    sensor_msgs::msg::BatteryState absent;
    absent.present = false;
    auto absent_deadline = std::chrono::steady_clock::now() + 3s;
    bool cleared = false;
    while (std::chrono::steady_clock::now() < absent_deadline) {
      publisher->publish(absent);
      executor.spin_some();
      {
        std::lock_guard<std::mutex> lock(*mutex);
        cleared = !sensors->has_batterystate();
        EXPECT_EQ(sensors->robot_info().robot_id(), "battery-test-robot");
      }
      if (cleared) break;
      std::this_thread::sleep_for(20ms);
    }
    ASSERT_TRUE(cleared);
  }
  ASSERT_TRUE(publish_and_wait(0.33f));

  // A new sample resets the expiration window: it must not disappear on
  // the timer tick that would have expired the previous 42% sample.
  const auto early_deadline = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < early_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  {
    std::lock_guard<std::mutex> lock(*mutex);
    EXPECT_TRUE(sensors->has_batterystate());
  }
}
