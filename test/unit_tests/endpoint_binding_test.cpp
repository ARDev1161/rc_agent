#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <thread>
#include <memory>
#include <stdexcept>
#include <vector>
#include "modules/basecontrol/basecontrol.h"

class EndpointBindingTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(EndpointBindingTest, LegacyDefaults)
{
    Base::BaseControl controls;
    std::mutex mutex;
    BaseControlNode node(&controls, mutex);
    EXPECT_EQ(node.get_subscriptions_info_by_topic("/odom").size(), 1u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/cmd_vel").size(), 1u);
}

TEST_F(EndpointBindingTest, AlternateRobotProfile)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/robot_b/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/robot_b/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist_stamped")
    });
    BaseControlNode node(&controls, mutex, options);
    EXPECT_EQ(node.get_subscriptions_info_by_topic("/robot_b/odom").size(), 1u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/robot_b/cmd_vel").size(), 1u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/cmd_vel").size(), 0u);
}

TEST_F(EndpointBindingTest, UnsupportedMessageTypeFailsClosed)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "unsupported")
    });
    EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
}

TEST_F(EndpointBindingTest, EmptyOdometryTopicFailsClosed)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "")
    });
    EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
}

TEST_F(EndpointBindingTest, EmptyVelocityTopicFailsClosed)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "")
    });
    EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
}

TEST_F(EndpointBindingTest, EmptyVelocityFrameFailsClosed)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.cmd_vel_frame_id", "")
    });
    EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
}


// P2: independent odometry subscriptions must not affect the other robot's
// motion-status state. This exercises real ROS2 delivery, not only topic names.
TEST_F(EndpointBindingTest, TwoProfilesKeepOdometryStatusIsolated)
{
    using Odom = nav_msgs::msg::Odometry;
    Base::BaseControl control_a;
    Base::BaseControl control_b;
    std::mutex mutex_a;
    std::mutex mutex_b;
    rclcpp::NodeOptions options_a;
    rclcpp::NodeOptions options_b;
    options_a.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/profile_a/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/profile_a/cmd_vel")
    });
    options_b.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/profile_b/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/profile_b/cmd_vel")
    });
    auto node_a = std::make_shared<BaseControlNode>(&control_a, mutex_a, options_a);
    auto node_b = std::make_shared<BaseControlNode>(&control_b, mutex_b, options_b);
    auto publisher_node = std::make_shared<rclcpp::Node>("two_profile_odom_publisher");
    auto publisher_a = publisher_node->create_publisher<Odom>("/profile_a/odom", 10);
    auto publisher_b = publisher_node->create_publisher<Odom>("/profile_b/odom", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_a);
    executor.add_node(node_b);
    executor.add_node(publisher_node);

    Odom moving;
    moving.twist.twist.linear.x = 0.5;
    Odom stationary;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node_a->getBaseControlStatus() != BaseControlNode::ControlStatus::RUNNING &&
           std::chrono::steady_clock::now() < deadline) {
        publisher_a->publish(moving);
        publisher_b->publish(stationary);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    ASSERT_TRUE(node_a->getBaseControlStatus() == BaseControlNode::ControlStatus::RUNNING);
    EXPECT_TRUE(node_b->getBaseControlStatus() == BaseControlNode::ControlStatus::READY_OK);

    // Only B moves now; A receives no new odometry and must retain its state.
    const auto second_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node_b->getBaseControlStatus() != BaseControlNode::ControlStatus::RUNNING &&
           std::chrono::steady_clock::now() < second_deadline) {
        publisher_b->publish(moving);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    ASSERT_TRUE(node_b->getBaseControlStatus() == BaseControlNode::ControlStatus::RUNNING);
    EXPECT_TRUE(node_a->getBaseControlStatus() == BaseControlNode::ControlStatus::RUNNING);
}


// P2: exercise actual ROS2 cmd_vel delivery on two independently configured
// adapters. This is topic routing only, not P3 gRPC session isolation or a
// v2 velocity-authority safety test.
TEST_F(EndpointBindingTest, TwoProfilesKeepVelocityPublishersIsolated)
{
    using Twist = geometry_msgs::msg::Twist;
    Base::BaseControl control_a;
    Base::BaseControl control_b;
    std::mutex mutex_a;
    std::mutex mutex_b;
    rclcpp::NodeOptions options_a;
    rclcpp::NodeOptions options_b;
    options_a.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/velocity_a/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/velocity_a/cmd_vel")
    });
    options_b.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/velocity_b/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/velocity_b/cmd_vel")
    });
    auto node_a = std::make_shared<BaseControlNode>(&control_a, mutex_a, options_a);
    auto node_b = std::make_shared<BaseControlNode>(&control_b, mutex_b, options_b);
    auto observer = std::make_shared<rclcpp::Node>("two_profile_velocity_observer");
    std::vector<double> received_a;
    std::vector<double> received_b;
    auto sub_a = observer->create_subscription<Twist>(
        "/velocity_a/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { received_a.push_back(msg->linear.x); });
    auto sub_b = observer->create_subscription<Twist>(
        "/velocity_b/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { received_b.push_back(msg->linear.x); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_a);
    executor.add_node(node_b);
    executor.add_node(observer);

    // Allow DDS discovery before asserting delivery.
    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((node_a->count_subscribers("/velocity_a/cmd_vel") == 0 ||
            node_b->count_subscribers("/velocity_b/cmd_vel") == 0) &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node_a->count_subscribers("/velocity_a/cmd_vel"), 0u);
    ASSERT_GT(node_b->count_subscribers("/velocity_b/cmd_vel"), 0u);

    Twist command_a;
    command_a.linear.x = 0.11;
    node_a->publishToBaseControlNode(command_a, "cmd_vel");
    const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received_a.empty() && std::chrono::steady_clock::now() < first_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received_a.size(), 1u);
    EXPECT_DOUBLE_EQ(received_a.front(), 0.11);
    EXPECT_TRUE(received_b.empty());

    Twist command_b;
    command_b.linear.x = -0.22;
    node_b->publishToBaseControlNode(command_b, "cmd_vel");
    const auto second_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received_b.empty() && std::chrono::steady_clock::now() < second_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received_b.size(), 1u);
    EXPECT_DOUBLE_EQ(received_b.front(), -0.22);
    EXPECT_EQ(received_a.size(), 1u);
}


// P2: mixed Twist/TwistStamped robot profiles must retain their own ROS
// message types, frame metadata and payloads. This is legacy topic binding,
// not authorization for v2 velocity commands.
TEST_F(EndpointBindingTest, MixedVelocityMessageTypesRemainIsolated)
{
    using Twist = geometry_msgs::msg::Twist;
    using Stamped = geometry_msgs::msg::TwistStamped;
    Base::BaseControl control_a;
    Base::BaseControl control_b;
    std::mutex mutex_a;
    std::mutex mutex_b;
    rclcpp::NodeOptions options_a;
    options_a.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/mixed_a/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/mixed_a/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist")
    });
    rclcpp::NodeOptions options_b;
    options_b.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/mixed_b/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/mixed_b/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_msg_type", "twist_stamped"),
        rclcpp::Parameter("basecontrol.cmd_vel_frame_id", "robot_b/base_link")
    });
    auto node_a = std::make_shared<BaseControlNode>(&control_a, mutex_a, options_a);
    auto node_b = std::make_shared<BaseControlNode>(&control_b, mutex_b, options_b);
    auto observer = std::make_shared<rclcpp::Node>("mixed_velocity_type_observer");
    std::vector<Twist> received_a;
    std::vector<Stamped> received_b;
    auto sub_a = observer->create_subscription<Twist>(
        "/mixed_a/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { received_a.push_back(*msg); });
    auto sub_b = observer->create_subscription<Stamped>(
        "/mixed_b/cmd_vel", 10,
        [&](const Stamped::SharedPtr msg) { received_b.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_a);
    executor.add_node(node_b);
    executor.add_node(observer);

    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((node_a->count_subscribers("/mixed_a/cmd_vel") == 0 ||
            node_b->count_subscribers("/mixed_b/cmd_vel") == 0) &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node_a->count_subscribers("/mixed_a/cmd_vel"), 0u);
    ASSERT_GT(node_b->count_subscribers("/mixed_b/cmd_vel"), 0u);

    Twist a;
    a.linear.x = 0.17;
    node_a->publishToBaseControlNode(a, "cmd_vel");
    const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received_a.empty() && std::chrono::steady_clock::now() < first_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received_a.size(), 1u);
    EXPECT_DOUBLE_EQ(received_a.front().linear.x, 0.17);
    EXPECT_TRUE(received_b.empty());

    Twist b;
    b.linear.x = -0.29;
    b.angular.z = 0.31;
    node_b->publishToBaseControlNode(b, "cmd_vel");
    const auto second_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received_b.empty() && std::chrono::steady_clock::now() < second_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received_b.size(), 1u);
    EXPECT_EQ(received_b.front().header.frame_id, "robot_b/base_link");
    EXPECT_GT(received_b.front().header.stamp.sec, 0);
    EXPECT_DOUBLE_EQ(received_b.front().twist.linear.x, -0.29);
    EXPECT_DOUBLE_EQ(received_b.front().twist.angular.z, 0.31);
    EXPECT_EQ(received_a.size(), 1u);
}


// P2: a remote selector must not turn an allowlisted velocity publisher into
// an arbitrary ROS topic publisher. Invalid selectors fail closed while the
// existing configured cmd_vel route remains operational.
TEST_F(EndpointBindingTest, UnknownVelocitySelectorCannotPublish)
{
    using Twist = geometry_msgs::msg::Twist;
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/selector_test/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/selector_test/cmd_vel")
    });
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("velocity_selector_observer");
    std::vector<double> received;
    auto sub = observer->create_subscription<Twist>(
        "/selector_test/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { received.push_back(msg->linear.x); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);

    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/selector_test/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node->count_subscribers("/selector_test/cmd_vel"), 0u);

    Twist command;
    command.linear.x = 0.42;
    node->publishToBaseControlNode(command, "/selector_test/cmd_vel");
    node->publishToBaseControlNode(command, "arbitrary_remote_topic");
    const auto reject_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < reject_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty());

    node->publishToBaseControlNode(command, "cmd_vel");
    const auto delivery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < delivery_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_DOUBLE_EQ(received.front(), 0.42);
}
