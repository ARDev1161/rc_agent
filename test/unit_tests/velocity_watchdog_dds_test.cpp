#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <limits>
#include <cstdint>
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
        // Arm with actual nonzero SI motion; the watchdog must publish zero,
        // not merely echo the last admitted command.
        command.mutable_linear()->set_x(0.2);
        command.mutable_angular()->set_z(-0.1);
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
    // Acknowledged stop is a one-shot event: subsequent watchdog ticks must
    // not emit duplicate zero commands on the same lease.
    const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < quiet_until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(received.size(), 1u);
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

// Verify timeout-driven stamped stop independently of explicit revoke.
TEST_F(WatchdogDdsTest, ExpiredLeasePublishesOneStampedZero)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_stamped_ttl/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_stamped_ttl/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist_stamped"),
        rclcpp::Parameter("basecontrol.cmd_vel_frame_id", "robot_c/base_link")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_stamped_ttl_observer");
    std::vector<geometry_msgs::msg::TwistStamped> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/watchdog_stamped_ttl/cmd_vel", 10,
        [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_stamped_ttl/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_stamped_ttl/cmd_vel"), 0u);
    VelocityWatchdogTestPeer::arm(*node, 100);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_EQ(received.front().header.frame_id, "robot_c/base_link");
    const auto& zero = received.front().twist;
    EXPECT_DOUBLE_EQ(zero.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(zero.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(zero.linear.z, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.y, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular.z, 0.0);
    EXPECT_FALSE(VelocityWatchdogTestPeer::pending(*node));
    const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < quiet_until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(received.size(), 1u);
}

// A lost transport session must revoke an already admitted nonzero command.
// This simulates disconnect at the gate; network disconnect wiring remains a
// separate integration gate and is not claimed by this fixture.
TEST_F(WatchdogDdsTest, DisconnectedSessionPublishesZeroTwist)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_disconnect/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_disconnect/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_disconnect_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/watchdog_disconnect/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_disconnect/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_disconnect/cmd_vel"), 0u);
    VelocityWatchdogTestPeer::arm(*node, 1000);
    VelocityWatchdogTestPeer::revoke(*node); // simulate authenticated session disconnect
    ASSERT_TRUE(VelocityWatchdogTestPeer::pending(*node));
    VelocityWatchdogTestPeer::grantNext(*node);
    ASSERT_TRUE(VelocityWatchdogTestPeer::expired(*node));
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

namespace {
using Gate = rc::api::v2::VelocityGate;
using namespace std::chrono_literals;

rc::api::v2::Velocity makeVelocity(std::uint64_t sequence, unsigned ttl_ms) {
    rc::api::v2::Velocity command;
    command.set_sequence(sequence);
    command.set_valid_for_ms(ttl_ms);
    command.set_frame_id("base_link");
    command.mutable_linear()->set_x(0.2);
    return command;
}
} // namespace

// Deterministic monotonic-clock checks complement the DDS delivery tests.
// These do not authenticate a network session or grant production teleop.
TEST(VelocityGateContract, ExpiryRequiresPublishedStopBeforeNewLease) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session-a", "lease-a", t0);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 100), "session-a", "lease-a", true, t0));
    EXPECT_FALSE(gate.stopPending(t0 + 99ms));
    EXPECT_TRUE(gate.stopPending(t0 + 100ms));
    gate.grant("session-b", "lease-b", t0 + 101ms);
    EXPECT_FALSE(gate.admit(makeVelocity(1, 100), "session-b", "lease-b", true, t0 + 101ms));
    EXPECT_TRUE(gate.stopPending(t0 + 102ms));
    gate.acknowledgeStopPublished();
    EXPECT_FALSE(gate.stopPending(t0 + 103ms));
    gate.grant("session-b", "lease-b", t0 + 104ms);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 100), "session-b", "lease-b", true, t0 + 104ms));
}

// A stale command arriving after TTL expiry must latch a stop even if the
// periodic watchdog has not yet observed the timeout. Regrant cannot bypass
// the pending zero publish; no real transport authorization is simulated.
TEST(VelocityGateContract, LateCommandBeforeWatchdogTickRequiresZeroPublish) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session-a", "lease-a", t0);
    ASSERT_TRUE(gate.admit(makeVelocity(1, 100), "session-a", "lease-a", true, t0));
    EXPECT_FALSE(gate.admit(makeVelocity(2, 100), "session-a", "lease-a", true, t0 + 100ms));
    EXPECT_TRUE(gate.stopPending(t0 + 101ms));
    gate.grant("session-b", "lease-b", t0 + 102ms);
    EXPECT_FALSE(gate.admit(makeVelocity(1, 100), "session-b", "lease-b", true, t0 + 102ms));
    EXPECT_TRUE(gate.stopPending(t0 + 103ms));
    gate.acknowledgeStopPublished();
    gate.grant("session-b", "lease-b", t0 + 104ms);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 100), "session-b", "lease-b", true, t0 + 104ms));
}

TEST(VelocityGateContract, ReplayedSequenceRevokesLeaseAndLatchesStop) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session", "lease", t0);
    EXPECT_TRUE(gate.admit(makeVelocity(7, 500), "session", "lease", true, t0));
    EXPECT_FALSE(gate.admit(makeVelocity(7, 500), "session", "lease", true, t0 + 1ms));
    EXPECT_TRUE(gate.stopPending(t0 + 2ms));
    EXPECT_TRUE(gate.expired(t0 + 2ms));
}

TEST(VelocityGateContract, MonotonicRollbackRevokesAfterAcceptedMotion) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session", "lease", t0);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 500), "session", "lease", true, t0 + 20ms));
    EXPECT_FALSE(gate.admit(makeVelocity(2, 500), "session", "lease", true, t0 + 19ms));
    EXPECT_TRUE(gate.stopPending(t0 + 21ms));
}

TEST(VelocityGateContract, InvalidNonFiniteMotionRevokesWithoutUnlockingStop) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session", "lease", t0);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 500), "session", "lease", true, t0));
    auto invalid = makeVelocity(2, 500);
    invalid.mutable_angular()->set_z(std::numeric_limits<double>::infinity());
    EXPECT_FALSE(gate.admit(invalid, "session", "lease", true, t0 + 1ms));
    EXPECT_TRUE(gate.stopPending(t0 + 2ms));
}

TEST(VelocityGateContract, WrongSessionRevokesMotionAndRequiresStopBeforeRegrant) {
    Gate gate;
    const auto t0 = Gate::Clock::time_point{} + 10s;
    gate.grant("session-a", "lease-a", t0);
    ASSERT_TRUE(gate.admit(makeVelocity(1, 500), "session-a", "lease-a", true, t0));
    EXPECT_FALSE(gate.admit(makeVelocity(2, 500), "session-b", "lease-a", true, t0 + 1ms));
    EXPECT_TRUE(gate.stopPending(t0 + 2ms));
    gate.grant("session-b", "lease-b", t0 + 3ms);
    EXPECT_FALSE(gate.admit(makeVelocity(1, 500), "session-b", "lease-b", true, t0 + 4ms));
    gate.acknowledgeStopPublished();
    gate.grant("session-b", "lease-b", t0 + 5ms);
    EXPECT_TRUE(gate.admit(makeVelocity(1, 500), "session-b", "lease-b", true, t0 + 5ms));
}

// A malformed replacement grant must revoke an already moving session and
// retain the zero-stop barrier until the ROS publisher acknowledges it.
// This covers disconnect/re-auth races without enabling a network grant API.
TEST(VelocityGateContract, EmptyReplacementCredentialsCannotKeepMotionAlive) {
    const auto t0 = Gate::Clock::time_point{} + 10s;
    for (const auto& credentials : {
             std::pair<std::string, std::string>{"", "lease-b"},
             std::pair<std::string, std::string>{"session-b", ""}}) {
        Gate gate;
        gate.grant("session-a", "lease-a", t0);
        ASSERT_TRUE(gate.admit(makeVelocity(1, 500), "session-a", "lease-a", true, t0));
        gate.grant(credentials.first, credentials.second, t0 + 1ms);
        EXPECT_TRUE(gate.stopPending(t0 + 2ms));
        EXPECT_FALSE(gate.admit(makeVelocity(2, 500), "session-a", "lease-a", true, t0 + 3ms));
        gate.grant("session-b", "lease-b", t0 + 4ms);
        EXPECT_FALSE(gate.admit(makeVelocity(1, 500), "session-b", "lease-b", true, t0 + 4ms));
        gate.acknowledgeStopPublished();
        gate.grant("session-b", "lease-b", t0 + 5ms);
        EXPECT_TRUE(gate.admit(makeVelocity(1, 500), "session-b", "lease-b", true, t0 + 5ms));
    }
}

TEST(VelocityGateContract, WrongLeaseAndInvalidTtlFailClosed) {
    const auto t0 = Gate::Clock::time_point{} + 10s;
    for (const auto invalid_ttl : {0u, 1001u}) {
        Gate gate;
        gate.grant("session", "lease", t0);
        ASSERT_TRUE(gate.admit(makeVelocity(1, 500), "session", "lease", true, t0));
        EXPECT_FALSE(gate.admit(makeVelocity(2, invalid_ttl), "session", "lease", true, t0 + 1ms));
        EXPECT_TRUE(gate.stopPending(t0 + 2ms));
    }
    Gate gate;
    gate.grant("session", "lease", t0);
    ASSERT_TRUE(gate.admit(makeVelocity(1, 500), "session", "lease", true, t0));
    EXPECT_FALSE(gate.admit(makeVelocity(2, 500), "session", "other-lease", true, t0 + 1ms));
    EXPECT_TRUE(gate.stopPending(t0 + 2ms));
}


TEST_F(WatchdogDdsTest, NonFiniteLocalVelocityIsRejectedOnConfiguredOutput)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_invalid/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_invalid/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_invalid_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/watchdog_invalid/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_invalid/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_invalid/cmd_vel"), 0u);

    // Every axis must reject NaN and both signs of infinity. A robot that
    // currently uses only x/z must still reject invalid unused axes.
    for (int axis = 0; axis < 6; ++axis) {
        for (const double invalid_value : {
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()}) {
            geometry_msgs::msg::Twist invalid;
            double* values[] = {&invalid.linear.x, &invalid.linear.y, &invalid.linear.z,
                                &invalid.angular.x, &invalid.angular.y, &invalid.angular.z};
            *values[axis] = invalid_value;
            node->publishToBaseControlNode(invalid, "cmd_vel");
        }
    }
    const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < quiet_until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty());
}


TEST_F(WatchdogDdsTest, NonFiniteStampedVelocityIsRejectedOnConfiguredOutput)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_invalid_stamped/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_invalid_stamped/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist_stamped"),
        rclcpp::Parameter("basecontrol.cmd_vel_frame_id", "robot_d/base_link")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_invalid_stamped_observer");
    std::vector<geometry_msgs::msg::TwistStamped> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/watchdog_invalid_stamped/cmd_vel", 10,
        [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_invalid_stamped/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_invalid_stamped/cmd_vel"), 0u);

    for (int axis = 0; axis < 6; ++axis) {
        for (const double invalid_value : {
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()}) {
            geometry_msgs::msg::Twist invalid;
            double* values[] = {&invalid.linear.x, &invalid.linear.y, &invalid.linear.z,
                                &invalid.angular.x, &invalid.angular.y, &invalid.angular.z};
            *values[axis] = invalid_value;
            node->publishToBaseControlNode(invalid, "cmd_vel");
        }
    }
    const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < quiet_until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty());
}

 
// Local ROS publication must preserve SI-valued six-axis velocities exactly.
// This exercises the configured DDS adapter, NOT remote teleop authorization.
TEST_F(WatchdogDdsTest, GenericLocalVelocityCannotBypassLease)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/watchdog_si/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/watchdog_si/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("watchdog_si_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/watchdog_si/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/watchdog_si/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/watchdog_si/cmd_vel"), 0u);
    geometry_msgs::msg::Twist command;
    command.linear.x = 0.25;
    command.linear.y = -0.125;
    command.linear.z = 0.0625;
    command.angular.x = -0.5;
    command.angular.y = 0.75;
    command.angular.z = -1.25;
    // The generic publisher has no authenticated v2 lease. Reject all six
    // nonzero SI components; an explicit zero-stop must still reach DDS.
    node->publishToBaseControlNode(command, "cmd_vel");
    const auto reject_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
    while (std::chrono::steady_clock::now() < reject_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(received.empty());

    geometry_msgs::msg::Twist zero;
    node->publishToBaseControlNode(zero, "cmd_vel");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_DOUBLE_EQ(received.front().linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received.front().linear.y, 0.0);
    EXPECT_DOUBLE_EQ(received.front().linear.z, 0.0);
    EXPECT_DOUBLE_EQ(received.front().angular.x, 0.0);
    EXPECT_DOUBLE_EQ(received.front().angular.y, 0.0);
    EXPECT_DOUBLE_EQ(received.front().angular.z, 0.0);
}

TEST_F(WatchdogDdsTest, RejectsAliasedVelocityOutputTopics)
{
    const auto verify_rejected = [](const std::string& primary,
                                    const std::string& smoothed,
                                    const std::string& navigation) {
        Base::BaseControl controls;
        std::mutex mutex;
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("basecontrol.odom_topic", "/alias_test/odom"),
            rclcpp::Parameter("basecontrol.cmd_vel_topic", primary),
            rclcpp::Parameter("basecontrol.cmd_vel_smoothed_topic", smoothed),
            rclcpp::Parameter("basecontrol.cmd_vel_nav_topic", navigation)});
        EXPECT_THROW((BaseControlNode(&controls, mutex, options)), std::invalid_argument);
    };
    verify_rejected("/alias_test/cmd", "/alias_test/cmd", "/alias_test/nav");
    verify_rejected("/alias_test/cmd", "/alias_test/smooth", "/alias_test/cmd");
    verify_rejected("/alias_test/cmd", "/alias_test/aux", "/alias_test/aux");
}


TEST_F(WatchdogDdsTest, DirectCmdVelCannotBypassLease)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/direct_guard/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/direct_guard/cmd_vel")});
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("direct_guard_observer");
    std::vector<geometry_msgs::msg::Twist> received;
    auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
        "/direct_guard/cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { received.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/direct_guard/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(node->count_subscribers("/direct_guard/cmd_vel"), 0u);

    geometry_msgs::msg::Twist unauthorized;
    unauthorized.linear.x = 0.25; // SI m/s, but no session/lease
    node->publishToBaseControlNode(unauthorized, "cmd_vel");
    const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < quiet_until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty()) << "unauthorized direct motion reached DDS";

    // Emergency zero is still allowed through the generic output path.
    geometry_msgs::msg::Twist zero;
    node->publishToBaseControlNode(zero, "cmd_vel");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_DOUBLE_EQ(received.front().linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received.front().angular.z, 0.0);
}
