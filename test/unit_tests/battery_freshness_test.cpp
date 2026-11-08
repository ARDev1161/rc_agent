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
  auto sensors = std::make_shared<Robot::Sensors>();
  auto mutex = std::make_shared<std::mutex>();
  auto battery = std::make_shared<BatteryStateNode>(
      sensors, mutex, "/rc_api_v2_test_battery", 350ms);
  auto publisher_node = std::make_shared<rclcpp::Node>("battery_test_publisher");
  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::BatteryState>(
      "/rc_api_v2_test_battery", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(battery);
  executor.add_node(publisher_node);

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
  ASSERT_TRUE(publish_and_wait(0.76f));
}
