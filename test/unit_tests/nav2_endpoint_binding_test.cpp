#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
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
    // An unavailable action completes locally without a goal handle. A late
    // operator cancel must be a no-op, not a cancellation of another goal.
    navigator.cancelTask();
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    EXPECT_TRUE(rclcpp::ok());

    // Every optional action must fail closed independently; no global ROS shutdown.
    navigator.goThroughPoses({pose});
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator.cancelTask();
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator.followWaypoints({pose});
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator.cancelTask();
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator.spin(0.5, 1.0);
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator.cancelTask();
    EXPECT_TRUE(navigator.isTaskComplete());
    EXPECT_TRUE(navigator.getResult() == rclcpp_action::ResultCode::UNKNOWN);
    EXPECT_TRUE(rclcpp::ok());

    // The configured localization publisher must use the override, not
    // silently publish to the legacy initialpose topic.
    geometry_msgs::msg::PoseWithCovarianceStamped initial;
    initial.header.frame_id = "map";
    navigator.setInitialPose(initial);
    const auto topics = navigator.get_topic_names_and_types();
    EXPECT_TRUE(topics.find("/offline_test/initialpose") != topics.end());
    EXPECT_TRUE(topics.find("/initialpose") == topics.end());
}

TEST_F(Nav2EndpointBindingTest, ConfiguredInitialPoseActuallyPublishes)
{
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.initial_pose_topic", "/nav2_binding_test/initialpose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_initialpose_publisher_test", request, response, grpc_mutex, options);
    auto observer = std::make_shared<rclcpp::Node>("nav2_initialpose_observer_test");
    using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
    bool received = false;
    double observed_x = 0.0;
    auto subscription = observer->create_subscription<Pose>(
        "/nav2_binding_test/initialpose", 10,
        [&](const Pose::SharedPtr msg) {
            observed_x = msg->pose.pose.position.x;
            received = true;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(observer);
    Pose pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 12.5;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        // Re-publish while DDS discovery is settling; this is a bounded
        // transport check, not merely a ROS graph topic-name assertion.
        navigator->setInitialPose(pose);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    EXPECT_TRUE(received);
    if (received) EXPECT_DOUBLE_EQ(observed_x, 12.5);
    EXPECT_TRUE(rclcpp::ok());
}


#include <rclcpp_action/create_server.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <atomic>
#include <functional>
#include <vector>

// Exercise actual ROS action transport, not just action-name resolution.
// The fake server is local to this test and never commands a robot.
TEST_F(Nav2EndpointBindingTest, MockNavigateToPoseSuccessAndRejection)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action", "/nav2_mock_test/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_mock_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_mock_server");
    std::atomic<bool> reject{false};
    std::atomic<int> accepted{0};
    std::atomic<int> cancel_requests{0};
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_test/navigate_to_pose",
        [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return reject.load() ? rclcpp_action::GoalResponse::REJECT
                                 : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<GoalHandle>) {
            ++cancel_requests;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<GoalHandle> handle) {
            ++accepted;
            handle->succeed(std::make_shared<Action::Result>());
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    // DDS discovery is asynchronous; wait until the server is visible.
    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    auto client = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_test/navigate_to_pose");
    while (!client->action_server_is_ready() &&
           std::chrono::steady_clock::now() < ready_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(client->action_server_is_ready());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    const auto success_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!navigator->isTaskComplete() &&
           std::chrono::steady_clock::now() < success_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(accepted.load(), 1);
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    navigator->cancelTask();
    for (int i = 0; i < 10; ++i) executor.spin_some();
    EXPECT_EQ(cancel_requests.load(), 0);
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);

    reject = true;
    navigator->goToPose(pose);
    EXPECT_FALSE(navigator->isTaskComplete());
    const auto reject_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!navigator->isTaskComplete() &&
           std::chrono::steady_clock::now() < reject_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(accepted.load(), 1);
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    EXPECT_TRUE(rclcpp::ok());
}


// Cancellation must not be reported as terminal until the action server
// acknowledges the request and returns a CANCELED result.
TEST_F(Nav2EndpointBindingTest, MockNavigateToPoseCancelWaitsForServerResult)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action", "/nav2_mock_cancel/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_cancel_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_cancel_server");
    std::shared_ptr<GoalHandle> active_goal;
    std::atomic<int> cancel_requests{0};
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_cancel/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<GoalHandle>) {
            ++cancel_requests;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<GoalHandle> handle) {
            // Do not finish immediately: cancellation is tested while pending.
            active_goal = handle;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_cancel/navigate_to_pose");
    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!observer->action_server_is_ready() &&
           std::chrono::steady_clock::now() < ready_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(observer->action_server_is_ready());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    const auto accept_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!active_goal && std::chrono::steady_clock::now() < accept_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(active_goal != nullptr);
    EXPECT_FALSE(navigator->isTaskComplete());

    navigator->cancelTask();
    // No executor progress means the backend has not acknowledged cancellation.
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);

    const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((!active_goal->is_canceling() || cancel_requests.load() == 0) &&
           std::chrono::steady_clock::now() < cancel_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(cancel_requests.load(), 0);
    ASSERT_TRUE(active_goal->is_canceling());
    EXPECT_FALSE(navigator->isTaskComplete());

    active_goal->canceled(std::make_shared<Action::Result>());
    const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!navigator->isTaskComplete() &&
           std::chrono::steady_clock::now() < result_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::CANCELED);
    EXPECT_TRUE(rclcpp::ok());
}


// A backend can reject cancellation while its goal remains executable.
// In that case the operator must observe the actual terminal action result,
// not a fabricated CANCELED status after requesting cancellation.
TEST_F(Nav2EndpointBindingTest, MockNavigateToPoseRejectedCancelCanStillSucceed)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_cancel_reject/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_cancel_reject_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_cancel_reject_server");
    std::shared_ptr<GoalHandle> active_goal;
    std::atomic<int> cancel_requests{0};
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_cancel_reject/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<GoalHandle>) {
            ++cancel_requests;
            return rclcpp_action::CancelResponse::REJECT;
        },
        [&](const std::shared_ptr<GoalHandle> handle) {
            active_goal = handle;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_cancel_reject/navigate_to_pose");
    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!observer->action_server_is_ready() &&
           std::chrono::steady_clock::now() < ready_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(observer->action_server_is_ready());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    const auto accept_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!active_goal && std::chrono::steady_clock::now() < accept_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(active_goal != nullptr);
    ASSERT_FALSE(navigator->isTaskComplete());

    navigator->cancelTask();
    EXPECT_FALSE(navigator->isTaskComplete());
    const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (cancel_requests.load() == 0 &&
           std::chrono::steady_clock::now() < cancel_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(cancel_requests.load(), 0);
    EXPECT_FALSE(active_goal->is_canceling());
    EXPECT_FALSE(navigator->isTaskComplete());

    active_goal->succeed(std::make_shared<Action::Result>());
    const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!navigator->isTaskComplete() &&
           std::chrono::steady_clock::now() < result_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(rclcpp::ok());
}

 
// A result from an earlier goal must never complete a newer goal.
TEST_F(Nav2EndpointBindingTest, LatePreviousGoalResultDoesNotCompleteReplacement)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action", "/nav2_mock_superseded/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_superseded_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_superseded_server");
    std::vector<std::shared_ptr<GoalHandle>> goals;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_superseded/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<GoalHandle> handle) { goals.push_back(handle); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_superseded/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped first;
    first.header.frame_id = "map";
    first.pose.position.x = 1.0;
    navigator->goToPose(first);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 1; }));
    geometry_msgs::msg::PoseStamped second = first;
    second.pose.position.x = 2.0;
    navigator->goToPose(second);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 2; }));
    EXPECT_FALSE(navigator->isTaskComplete());
    // An old goal may still publish feedback after a replacement. The new
    // navigator state must not display the superseded robot position.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    auto old_feedback = std::make_shared<Action::Feedback>();
    old_feedback->current_pose.pose.position.x = 99.0;
    goals[0]->publish_feedback(old_feedback);
    for (int i = 0; i < 30; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_NE(navigator->getCurrentPose().position.x, 99.0);
    auto new_feedback = std::make_shared<Action::Feedback>();
    new_feedback->current_pose.pose.position.x = 2.0;
    goals[1]->publish_feedback(new_feedback);
    const auto feedback_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (navigator->getCurrentPose().position.x != 2.0 &&
           std::chrono::steady_clock::now() < feedback_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_DOUBLE_EQ(navigator->getCurrentPose().position.x, 2.0);
    goals[0]->succeed(std::make_shared<Action::Result>());
    // Allow the stale result callback to run before inspecting state.
    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    goals[1]->succeed(std::make_shared<Action::Result>());
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
}


// A rejected replacement must remain rejected even if the prior goal succeeds late.
TEST_F(Nav2EndpointBindingTest, LatePreviousSuccessCannotOverrideRejectedReplacement)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_replacement_rejected/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_replacement_rejected_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_replacement_rejected_server");
    std::shared_ptr<GoalHandle> first_goal;
    int goal_requests = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_replacement_rejected/navigate_to_pose",
        [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return ++goal_requests == 1
                ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                : rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<GoalHandle> handle) { first_goal = handle; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_replacement_rejected/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped first;
    first.header.frame_id = "map";
    first.pose.position.x = 1.0;
    navigator->goToPose(first);
    ASSERT_TRUE(spin_until([&] { return first_goal != nullptr; }));
    geometry_msgs::msg::PoseStamped second = first;
    second.pose.position.x = 2.0;
    navigator->goToPose(second);
    ASSERT_TRUE(spin_until([&] { return goal_requests == 2; }));
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);

    first_goal->succeed(std::make_shared<Action::Result>());
    for (int i = 0; i < 30; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
}


// A replacement must not cause cancelTask() to cancel an earlier goal still
// running on the same Nav2 action server.
TEST_F(Nav2EndpointBindingTest, CancelOnlyCurrentAcceptedGoal)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action", "/nav2_mock_scoped_cancel/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_scoped_cancel_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_scoped_cancel_server");
    std::vector<std::shared_ptr<Handle>> goals;
    int canceled_first = 0;
    int canceled_second = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_scoped_cancel/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<Handle> handle) {
            if (goals.size() > 0 && handle->get_goal_id() == goals[0]->get_goal_id())
                ++canceled_first;
            if (goals.size() > 1 && handle->get_goal_id() == goals[1]->get_goal_id())
                ++canceled_second;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) { goals.push_back(handle); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_scoped_cancel/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 1; }));
    // Spin until the first goal-response callback has installed its handle.
    for (int i = 0; i < 20; ++i) executor.spin_some();
    pose.pose.position.x = 2.0;
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 2; }));
    for (int i = 0; i < 20; ++i) executor.spin_some();
    navigator->cancelTask();
    ASSERT_TRUE(spin_until([&] { return canceled_second == 1; }));
    EXPECT_EQ(canceled_first, 0);
    EXPECT_EQ(canceled_second, 1);
    // A repeated operator click must not send another backend cancellation.
    navigator->cancelTask();
    for (int i = 0; i < 20; ++i) executor.spin_some();
    EXPECT_EQ(canceled_first, 0);
    EXPECT_EQ(canceled_second, 1);
    EXPECT_FALSE(navigator->isTaskComplete()); // Cancel request is not a result.
}


// Cancel may arrive before DDS delivers the goal-response callback. It must
// remain pending and cancel precisely the newly accepted goal, not be lost.
TEST_F(Nav2EndpointBindingTest, CancelBeforeGoalResponseIsDelivered)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_early_cancel/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_early_cancel_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_early_cancel_server");
    std::shared_ptr<Handle> accepted_goal;
    int cancel_requests = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_early_cancel/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<Handle> handle) {
            ++cancel_requests;
            EXPECT_EQ(handle->get_goal_id(), accepted_goal->get_goal_id());
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) { accepted_goal = handle; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_early_cancel/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    // Deliberately do not spin the executor between send and cancel.
    navigator->cancelTask();
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    ASSERT_TRUE(spin_until([&] { return accepted_goal != nullptr; }));
    ASSERT_TRUE(spin_until([&] { return cancel_requests == 1; }));
    EXPECT_TRUE(accepted_goal->is_canceling());
    EXPECT_FALSE(navigator->isTaskComplete());
    accepted_goal->canceled(std::make_shared<Action::Result>());
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(cancel_requests, 1);
}


// A terminal success racing a pending cancel must never be rewritten as CANCELED.
// The server finishes before the executor processes the cancel response.
TEST_F(Nav2EndpointBindingTest, TerminalSuccessWinsPendingCancellation)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_cancel_result_race/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_cancel_result_race_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_cancel_result_race_server");
    std::shared_ptr<Handle> accepted_goal;
    int cancel_requests = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_cancel_result_race/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<Handle>) {
            ++cancel_requests;
            return rclcpp_action::CancelResponse::REJECT;
        },
        [&](const std::shared_ptr<Handle> handle) { accepted_goal = handle; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_cancel_result_race/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] { return accepted_goal != nullptr; }));
    // Ensure the goal-response callback has installed the current handle.
    for (int i = 0; i < 20; ++i) executor.spin_some();
    ASSERT_FALSE(navigator->isTaskComplete());
    navigator->cancelTask();
    EXPECT_FALSE(navigator->isTaskComplete());
    // No executor spin: publish a real terminal result while cancellation is pending.
    accepted_goal->succeed(std::make_shared<Action::Result>());
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    for (int i = 0; i < 20; ++i) executor.spin_some();
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_LE(cancel_requests, 1);
}


// An early cancel belongs to the generation that received it. Replacing that
// pending goal before its response must not cancel either the old or new goal.
TEST_F(Nav2EndpointBindingTest, PendingCancellationDoesNotLeakAcrossReplacement)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_pending_replacement/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_pending_replacement_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_pending_replacement_server");
    std::vector<std::shared_ptr<Handle>> goals;
    int cancel_requests = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_pending_replacement/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<Handle>) {
            ++cancel_requests;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) { goals.push_back(handle); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_pending_replacement/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    navigator->goToPose(pose);
    navigator->cancelTask();  // No executor spin: first response is still pending.
    pose.pose.position.x = 2.0;
    navigator->goToPose(pose);  // Supersedes the pending cancellation.
    ASSERT_TRUE(spin_until([&] { return goals.size() == 2; }));
    for (int i = 0; i < 30; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(cancel_requests, 0);
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    goals[0]->succeed(std::make_shared<Action::Result>());
    for (int i = 0; i < 20; ++i) executor.spin_some();
    EXPECT_FALSE(navigator->isTaskComplete());
    goals[1]->succeed(std::make_shared<Action::Result>());
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(cancel_requests, 0);
}


// A rejected goal has no result callback. A later accepted goal on the SAME
// navigator must clear the rejected terminal state and finish successfully.
TEST_F(Nav2EndpointBindingTest, RejectedGoalThenAcceptedGoalSucceeds)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_reject_recover/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_reject_recover_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_reject_recover_server");
    int requests_seen = 0;
    int accepted = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_reject_recover/navigate_to_pose",
        [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            ++requests_seen;
            return requests_seen == 1
                ? rclcpp_action::GoalResponse::REJECT
                : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<Handle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) {
            ++accepted;
            handle->succeed(std::make_shared<Action::Result>());
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_reject_recover/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] {
        return requests_seen == 1 && navigator->isTaskComplete();
    }));
    EXPECT_EQ(accepted, 0);
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    navigator->cancelTask(); // Rejected goal has nothing to cancel.
    EXPECT_TRUE(navigator->isTaskComplete());

    pose.pose.position.x = 2.0;
    navigator->goToPose(pose);
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    ASSERT_TRUE(spin_until([&] {
        return accepted == 1 && navigator->isTaskComplete();
    }));
    EXPECT_EQ(requests_seen, 2);
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(rclcpp::ok());
}


// A missing optional Nav2 server must not poison later requests. This tests
// recovery on the SAME navigator after the action server becomes available.
TEST_F(Nav2EndpointBindingTest, OfflineGoalThenServerAppearsAndNextGoalSucceeds)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_late_start/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_late_start_client", request, response, grpc_mutex, options);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    navigator->goToPose(pose);
    EXPECT_TRUE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    EXPECT_TRUE(rclcpp::ok());

    auto server_node = std::make_shared<rclcpp::Node>("nav2_late_start_server");
    int accepted = 0;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_late_start/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<Handle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) {
            ++accepted;
            handle->succeed(std::make_shared<Action::Result>());
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_late_start/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    pose.pose.position.x = 3.0;
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] {
        return accepted == 1 && navigator->isTaskComplete();
    }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(rclcpp::ok());
}


// A stale CANCELED result must not mark a replacement goal complete.
// This is distinct from a stale SUCCEEDED result: cancellation callbacks may
// arrive after the operator has already issued a new Nav2 command.
TEST_F(Nav2EndpointBindingTest, LatePreviousCancellationCannotCompleteReplacement)
{
    using Action = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    auto request = std::make_shared<Navigation::NavCommandRequest>();
    auto response = std::make_shared<Navigation::NavCommandResponse>();
    std::mutex grpc_mutex;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("nav2.navigate_to_pose_action",
                          "/nav2_mock_stale_cancel/navigate_to_pose")
    });
    auto navigator = std::make_shared<RobotNavigator>(
        "nav2_stale_cancel_client", request, response, grpc_mutex, options);
    auto server_node = std::make_shared<rclcpp::Node>("nav2_stale_cancel_server");
    std::vector<std::shared_ptr<Handle>> goals;
    std::vector<std::shared_ptr<Handle>> canceled_handles;
    auto server = rclcpp_action::create_server<Action>(
        server_node, "/nav2_mock_stale_cancel/navigate_to_pose",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<Handle> handle) {
            canceled_handles.push_back(handle);
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle) { goals.push_back(handle); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigator);
    executor.add_node(server_node);
    auto observer = rclcpp_action::create_client<Action>(
        server_node, "/nav2_mock_stale_cancel/navigate_to_pose");
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] { return observer->action_server_is_ready(); }));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 1; }));
    // Ensure the first goal response has installed the handle before cancel.
    for (int i = 0; i < 20; ++i) executor.spin_some();
    navigator->cancelTask();
    ASSERT_TRUE(spin_until([&] { return goals[0]->is_canceling(); }));
    pose.pose.position.x = 2.0;
    navigator->goToPose(pose);
    ASSERT_TRUE(spin_until([&] { return goals.size() == 2; }));
    for (int i = 0; i < 20; ++i) executor.spin_some();
    EXPECT_FALSE(navigator->isTaskComplete());
    // The first backend now reports the cancellation after replacement.
    goals[0]->canceled(std::make_shared<Action::Result>());
    for (int i = 0; i < 30; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(navigator->isTaskComplete());
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::UNKNOWN);
    goals[1]->succeed(std::make_shared<Action::Result>());
    ASSERT_TRUE(spin_until([&] { return navigator->isTaskComplete(); }));
    EXPECT_TRUE(navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_EQ(canceled_handles.size(), 1u);
    EXPECT_EQ(canceled_handles.front(), goals[0]);
    EXPECT_NE(canceled_handles.front(), goals[1]);
}
