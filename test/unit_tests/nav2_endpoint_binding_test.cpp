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

TEST_F(Nav2EndpointBindingTest, MissingNav2ServersDoNotShutdownRosContext)
{
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action", "/offline_test/navigate_to_pose"),
        rclcpp::Parameter("nav2.navigate_through_poses_action", "/offline_test/navigate_through_poses"),
        rclcpp::Parameter("nav2.follow_waypoints_action", "/offline_test/follow_waypoints"),
        rclcpp::Parameter("nav2.spin_action", "/offline_test/spin"),
        rclcpp::Parameter("nav2.initial_pose_topic", "/offline_test/initialpose")
    });
    RobotNavigator navigator("nav2_offline_test", request, response, grpc_mutex, options);
    EXPECT_TRUE(rclcpp::ok());
    EXPECT_EQ(navigator.get_parameter("nav2.navigate_to_pose_action").as_string(),
              "/offline_test/navigate_to_pose");

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator.goToPose(pose);
    EXPECT_TRUE(navigator.isTaskComplete());
    // ROS2 Humble gtest_vendor and the host GTest printer may differ; avoid
    // instantiating an enum printer that is not linked by the vendor library.
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    EXPECT_TRUE(rclcpp::ok());
}
