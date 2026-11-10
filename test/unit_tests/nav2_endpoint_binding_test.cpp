#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <string>
#include "modules/nav2/robot_navigator.h"

class Nav2EndpointBindingTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// These tests exercise actual ROS2 NodeOptions parameter resolution and
// fail-closed validation before waiting for any external Nav2 action server.
TEST_F(Nav2EndpointBindingTest, EmptyActionOrInitialPoseEndpointRejected)
{
    const std::string names[] = {
        "nav2.navigate_to_pose_action",
        "nav2.navigate_through_poses_action",
        "nav2.follow_waypoints_action",
        "nav2.spin_action",
        "nav2.initial_pose_topic"
    };
    for (const auto &name : names) {
        SCOPED_TRACE(name);
        auto request = std::make_shared<Navigation::NavCommandRequest>();
        auto response = std::make_shared<Navigation::NavCommandResponse>();
        std::mutex grpc_mutex;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(name, "")});
        EXPECT_THROW(
            RobotNavigator("nav2_endpoint_validation", request, response,
                           grpc_mutex, options),
            std::invalid_argument);
    }
}
