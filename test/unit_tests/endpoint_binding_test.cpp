#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <thread>
#include <memory>
#include <stdexcept>
#include <limits>
#include <utility>
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


// P2: auxiliary legacy velocity publishers must also be robot-profile local.
// Keep their existing selector names, but never route them to another robot's
// hardcoded global ROS topics when an explicit local binding is configured.
TEST_F(EndpointBindingTest, AuxiliaryVelocityTopicsFollowLocalProfile)
{
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/aux_profile/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/aux_profile/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_smoothed_topic", "/aux_profile/cmd_vel_smoothed"),
        rclcpp::Parameter("basecontrol.cmd_vel_nav_topic", "/aux_profile/cmd_vel_nav")
    });
    BaseControlNode node(&controls, mutex, options);
    EXPECT_EQ(node.get_publishers_info_by_topic("/aux_profile/cmd_vel_smoothed").size(), 1u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/aux_profile/cmd_vel_nav").size(), 1u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/cmd_vel_smoothed").size(), 0u);
    EXPECT_EQ(node.get_publishers_info_by_topic("/cmd_vel_nav").size(), 0u);
}

TEST_F(EndpointBindingTest, EmptyAuxiliaryVelocityTopicsFailClosed)
{
    for (const auto &name : {"basecontrol.cmd_vel_smoothed_topic",
                             "basecontrol.cmd_vel_nav_topic"}) {
        SCOPED_TRACE(name);
        Base::BaseControl controls;
        std::mutex mutex;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(name, "")});
        EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
    }
}

// P2: invalid local velocity scaling parameters must fail before any ROS
// publisher is created, rather than producing NaN or unbounded commands.
TEST_F(EndpointBindingTest, InvalidVelocityScalingParametersFailClosed)
{
    const std::vector<std::pair<std::string, double>> invalid = {
        {"basecontrol.input_min", std::numeric_limits<double>::quiet_NaN()},
        {"basecontrol.input_max", std::numeric_limits<double>::infinity()},
        {"basecontrol.max_linear_speed", -0.1},
        {"basecontrol.max_angular_speed", -0.1},
        {"basecontrol.input_deadzone", -1.0},
        {"basecontrol.max_linear_speed", std::numeric_limits<double>::quiet_NaN()},
        {"basecontrol.max_angular_speed", std::numeric_limits<double>::infinity()}
    };
    for (const auto &entry : invalid) {
        SCOPED_TRACE(entry.first);
        Base::BaseControl controls;
        std::mutex mutex;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(entry.first, entry.second)});
        EXPECT_THROW(BaseControlNode(&controls, mutex, options), std::invalid_argument);
    }
}


// P2: configured auxiliary outputs are independently routed on the ROS graph,
// not merely registered with the expected topic names. This does not enable
// v2 teleop or bypass the existing local selector allowlist.
TEST_F(EndpointBindingTest, AuxiliaryVelocityDeliveryRemainsProfileLocal)
{
    using Twist = geometry_msgs::msg::Twist;
    Base::BaseControl controls_a, controls_b;
    std::mutex mutex_a, mutex_b;
    rclcpp::NodeOptions options_a, options_b;
    options_a.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/aux_a/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/aux_a/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_smoothed_topic", "/aux_a/smoothed"),
        rclcpp::Parameter("basecontrol.cmd_vel_nav_topic", "/aux_a/nav")
    });
    options_b.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/aux_b/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/aux_b/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_smoothed_topic", "/aux_b/smoothed"),
        rclcpp::Parameter("basecontrol.cmd_vel_nav_topic", "/aux_b/nav")
    });
    auto a = std::make_shared<BaseControlNode>(&controls_a, mutex_a, options_a);
    auto b = std::make_shared<BaseControlNode>(&controls_b, mutex_b, options_b);
    auto observer = std::make_shared<rclcpp::Node>("auxiliary_velocity_observer");
    std::vector<double> a_smoothed, a_nav, b_smoothed, b_nav;
    auto as = observer->create_subscription<Twist>("/aux_a/smoothed", 10,
        [&](const Twist::SharedPtr m) { a_smoothed.push_back(m->linear.x); });
    auto an = observer->create_subscription<Twist>("/aux_a/nav", 10,
        [&](const Twist::SharedPtr m) { a_nav.push_back(m->linear.x); });
    auto bs = observer->create_subscription<Twist>("/aux_b/smoothed", 10,
        [&](const Twist::SharedPtr m) { b_smoothed.push_back(m->linear.x); });
    auto bn = observer->create_subscription<Twist>("/aux_b/nav", 10,
        [&](const Twist::SharedPtr m) { b_nav.push_back(m->linear.x); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(a);
    executor.add_node(b);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((a->count_subscribers("/aux_a/smoothed") == 0 ||
            a->count_subscribers("/aux_a/nav") == 0 ||
            b->count_subscribers("/aux_b/smoothed") == 0 ||
            b->count_subscribers("/aux_b/nav") == 0) &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(a->count_subscribers("/aux_a/smoothed"), 0u);
    ASSERT_GT(a->count_subscribers("/aux_a/nav"), 0u);
    ASSERT_GT(b->count_subscribers("/aux_b/smoothed"), 0u);
    ASSERT_GT(b->count_subscribers("/aux_b/nav"), 0u);

    Twist first;
    first.linear.x = 0.13;
    a->publishToBaseControlNode(first, "cmd_vel_smoothed");
    const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (a_smoothed.empty() && std::chrono::steady_clock::now() < first_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(a_smoothed.size(), 1u);
    EXPECT_DOUBLE_EQ(a_smoothed.front(), 0.13);
    EXPECT_TRUE(a_nav.empty());
    EXPECT_TRUE(b_smoothed.empty());
    EXPECT_TRUE(b_nav.empty());

    Twist second;
    second.linear.x = -0.24;
    b->publishToBaseControlNode(second, "cmd_vel_unsafety");
    const auto second_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (b_nav.empty() && std::chrono::steady_clock::now() < second_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(b_nav.size(), 1u);
    EXPECT_DOUBLE_EQ(b_nav.front(), -0.24);
    EXPECT_EQ(a_smoothed.size(), 1u);
    EXPECT_TRUE(a_nav.empty());
    EXPECT_TRUE(b_smoothed.empty());
}


// P2: malformed legacy velocity values must never reach any ROS output,
// including the auxiliary selectors. A subsequent finite command still works.
TEST_F(EndpointBindingTest, NonfiniteVelocityCommandsAreNotPublished)
{
    using Twist = geometry_msgs::msg::Twist;
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/finite_guard/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/finite_guard/cmd_vel"),
        rclcpp::Parameter("basecontrol.cmd_vel_smoothed_topic", "/finite_guard/smoothed"),
        rclcpp::Parameter("basecontrol.cmd_vel_nav_topic", "/finite_guard/nav")
    });
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("finite_velocity_guard_observer");
    std::vector<double> main_received, smooth_received, nav_received;
    auto main_sub = observer->create_subscription<Twist>("/finite_guard/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { main_received.push_back(msg->linear.x); });
    auto smooth_sub = observer->create_subscription<Twist>("/finite_guard/smoothed", 10,
        [&](const Twist::SharedPtr msg) { smooth_received.push_back(msg->linear.x); });
    auto nav_sub = observer->create_subscription<Twist>("/finite_guard/nav", 10,
        [&](const Twist::SharedPtr msg) { nav_received.push_back(msg->linear.x); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((node->count_subscribers("/finite_guard/cmd_vel") == 0 ||
            node->count_subscribers("/finite_guard/smoothed") == 0 ||
            node->count_subscribers("/finite_guard/nav") == 0) &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node->count_subscribers("/finite_guard/cmd_vel"), 0u);
    ASSERT_GT(node->count_subscribers("/finite_guard/smoothed"), 0u);
    ASSERT_GT(node->count_subscribers("/finite_guard/nav"), 0u);

    Twist malformed;
    malformed.linear.x = std::numeric_limits<double>::quiet_NaN();
    for (const auto &selector : {"cmd_vel", "cmd_vel_smoothed", "cmd_vel_unsafety"}) {
        node->publishToBaseControlNode(malformed, selector);
    }
    // Exercise every component and every allowed selector, not only linear.x.
    // The guard must reject the whole command even when the other five values
    // are finite, and must never publish a partially sanitized command.
    const auto invalid = std::numeric_limits<double>::infinity();
    for (int component = 0; component < 6; ++component) {
        Twist bad;
        bad.linear.x = 0.1;
        bad.linear.y = 0.2;
        bad.linear.z = 0.3;
        bad.angular.x = 0.4;
        bad.angular.y = 0.5;
        bad.angular.z = 0.6;
        switch (component) {
        case 0: bad.linear.x = invalid; break;
        case 1: bad.linear.y = invalid; break;
        case 2: bad.linear.z = invalid; break;
        case 3: bad.angular.x = invalid; break;
        case 4: bad.angular.y = invalid; break;
        case 5: bad.angular.z = invalid; break;
        }
        for (const auto &selector : {"cmd_vel", "cmd_vel_smoothed", "cmd_vel_unsafety"}) {
            node->publishToBaseControlNode(bad, selector);
        }
    }
    const auto settle = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < settle) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(main_received.empty());
    EXPECT_TRUE(smooth_received.empty());
    EXPECT_TRUE(nav_received.empty());

    Twist valid;
    valid.linear.x = 0.21;
    node->publishToBaseControlNode(valid, "cmd_vel");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (main_received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(main_received.size(), 1u);
    EXPECT_DOUBLE_EQ(main_received.front(), 0.21);
    EXPECT_TRUE(smooth_received.empty());
    EXPECT_TRUE(nav_received.empty());
}


// P2: malformed values arriving through the legacy protobuf controls must
// not poison change tracking; a subsequent finite command must still publish.
TEST_F(EndpointBindingTest, NonfiniteProtobufVelocityDoesNotBlockRecovery)
{
    using Twist = geometry_msgs::msg::Twist;
    Base::BaseControl controls;
    std::mutex mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("basecontrol.odom_topic", "/proto_guard/odom"),
        rclcpp::Parameter("basecontrol.cmd_vel_topic", "/proto_guard/cmd_vel")
    });
    auto node = std::make_shared<BaseControlNode>(&controls, mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("proto_velocity_guard_observer");
    std::vector<double> received;
    auto subscription = observer->create_subscription<Twist>(
        "/proto_guard/cmd_vel", 10,
        [&](const Twist::SharedPtr msg) { received.push_back(msg->linear.x); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers("/proto_guard/cmd_vel") == 0 &&
           std::chrono::steady_clock::now() < discovery) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node->count_subscribers("/proto_guard/cmd_vel"), 0u);

    {
        std::lock_guard<std::mutex> lock(mutex);
        controls.set_linearvelocity(std::numeric_limits<double>::quiet_NaN());
        controls.set_angularvelocity(0.0);
    }
    const auto settle = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < settle) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty());

    // An invalid angular component must also leave the last valid delta state
    // untouched. This is a separate protobuf input path from linear NaN.
    {
        std::lock_guard<std::mutex> lock(mutex);
        controls.set_linearvelocity(0.0);
        controls.set_angularvelocity(std::numeric_limits<double>::infinity());
    }
    const auto angular_settle = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < angular_settle) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received.empty());

    {
        std::lock_guard<std::mutex> lock(mutex);
        controls.set_linearvelocity(128.0);
        controls.set_angularvelocity(0.0);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(received.size(), 1u);
    EXPECT_GT(received.front(), 0.0);
}
