#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <thread>
#include <memory>
#include <stdexcept>
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
