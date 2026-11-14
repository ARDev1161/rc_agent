#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include "modules/basecontrol/basecontrol.h"

// The production gate is intentionally inaccessible to network callers.
// This fixture simulates a granted authenticated lease without exposing a
// public bypass, then observes the real ROS2 DDS watchdog publisher.
class VelocityWatchdogTestPeer {
public:
    static void arm(BaseControlNode& node, unsigned ttl_ms) {
        using Gate = rc::api::v2::VelocityGate;
        const auto now = Gate::Clock::now();
        node.velocity_gate_.grant("test-session", "test-lease", now);
        rc::api::v2::Velocity command;
        command.set_sequence(1);
        command.set_valid_for_ms(ttl_ms);
        command.set_frame_id("base_link");
        ASSERT_TRUE(node.velocity_gate_.admit(
            command, "test-session", "test-lease", true, now));
    }
    static bool pending(BaseControlNode& node) {
        return node.velocity_gate_.stopPending(rc::api::v2::VelocityGate::Clock::now());
    }
    static void revoke(BaseControlNode& node) { node.velocity_gate_.revoke(); }
    static bool rejectReleasedDeadman(BaseControlNode& node) {
        rc::api::v2::Velocity command;
        command.set_sequence(2);
        command.set_valid_for_ms(1000);
        command.set_frame_id("base_link");
        return node.velocity_gate_.admit(
            command, "test-session", "test-lease", false,
            rc::api::v2::VelocityGate::Clock::now());
    }
    static void grantNext(BaseControlNode& node) {
        node.velocity_gate_.grant("next-session", "next-lease",
                                  rc::api::v2::VelocityGate::Clock::now());
    }
    static bool expired(BaseControlNode& node) {
        return node.velocity_gate_.expired(rc::api::v2::VelocityGate::Clock::now());
    }
};

class WatchdogDdsTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(WatchdogDdsTest, ExpiredLeasePublishesZeroTwist)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_plain/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_plain/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_plain_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/watchdog_plain/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_plain/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_plain/cmd_vel"), 0u);
    VelocityWatchdogTestPeer::arm(*node, 100);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    const auto& zero = received.front();
    EXPECT_DOUBLE_EQ(zero.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(zero.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(zero.linear.z, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.y, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.z, 0.0);
    EXPECT_FALSE(VelocityWatchdogTestPeer::pending(*node));
    VelocityWatchdogTestPeer::grantNext(*node);
    EXPECT_TRUE(VelocityWatchdogTestPeer::expired(*node)); // no new motion admitted
}

TEST_F(WatchdogDdsTest, ExplicitRevokePublishesStampedZeroWithConfiguredFrame)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_stamped/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_stamped/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist_stamped"),
        rclcpp::Parameter("basecontrol.cmd_vel_frame_id", "robot_b/base_link")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_stamped_observer");
    std::vector<geometry_msgs::msg::TwistStamped> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/watchdog_stamped/cmd_vel", 10,
        [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_stamped/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_stamped/cmd_vel"), 0u);
    VelocityWatchdogTestPeer::arm(*node, 1000);
    VelocityWatchdogTestPeer::revoke(*node);
    VelocityWatchdogTestPeer::grantNext(*node);
    EXPECT_TRUE(VelocityWatchdogTestPeer::expired(*node));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_EQ(received.front().header.frame_id, "robot_b/base_link");
    EXPECT_DOUBLE_EQ(received.front().twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received.front().twist.angular.z, 0.0);
    EXPECT_FALSE(VelocityWatchdogTestPeer::pending(*node));
}

TEST_F(WatchdogDdsTest, ReleasedDeadmanLatchesStopUntilDdsPublish)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_deadman/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_deadman/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_deadman_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/watchdog_deadman/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_deadman/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_deadman/cmd_vel"), 0u);
    VelocityWatchdogTestPeer::arm(*node, 1000);
    ASSERT_FALSE(VelocityWatchdogTestPeer::rejectReleasedDeadman(*node));
    ASSERT_TRUE(VelocityWatchdogTestPeer::pending(*node));
    VelocityWatchdogTestPeer::grantNext(*node);
    EXPECT_TRUE(VelocityWatchdogTestPeer::pending(*node));
    EXPECT_TRUE(VelocityWatchdogTestPeer::expired(*node));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_DOUBLE_EQ(received.front().linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received.front().angular.z, 0.0);
    EXPECT_FALSE(VelocityWatchdogTestPeer::pending(*node));
}
