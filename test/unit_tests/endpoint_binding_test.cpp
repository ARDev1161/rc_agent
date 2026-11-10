#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
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
