#include <rclcpp/rclcpp.hpp>
#include "robot_navigator.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

void waitForCompletion(std::shared_ptr<BasicNavigator> navigator) {
    while (!navigator->isTaskComplete()) {
        rclcpp::spin_some(navigator);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto navigator = std::make_shared<BasicNavigator>("basic_navigator_node");

    RCLCPP_INFO(navigator->get_logger(), "Starting navigation test...");

    // Set initial pose
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = navigator->now();
    initial_pose.pose.pose.position.x = -1.0;
    initial_pose.pose.pose.position.y = 1.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;
    navigator->setInitialPose(initial_pose);
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(navigator->get_logger(), "Initial pose set.");

    // Send goal
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = navigator->now();
    goal_pose.pose.position.x = -1.0;
    goal_pose.pose.position.y = 3.55;
    goal_pose.pose.orientation.z = 0.75;
    goal_pose.pose.orientation.w = 0.7;

    navigator->goToPose(goal_pose);

    RCLCPP_INFO(navigator->get_logger(), "Sent goal to position (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);

    waitForCompletion(navigator);

    if (navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(navigator->get_logger(), "Goal reached successfully, proceeding to next task.");
    } else {
        RCLCPP_ERROR(navigator->get_logger(), "Goal failed!");
        rclcpp::shutdown();
        return 1;
    }

    // Check several point (goThroughPoses)
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    geometry_msgs::msg::PoseStamped pose1, pose2;

    pose1.header.frame_id = "map";
    pose1.pose.position.x = -1.0;
    pose1.pose.position.y = 1.0;
    pose1.pose.orientation.z = 1.0;
    pose1.pose.orientation.w = -0.1;

    pose2.header.frame_id = "map";
    pose2.pose.position.x = -5.55;
    pose2.pose.position.y = 3.0;
    pose2.pose.orientation.z = -0.53;
    pose2.pose.orientation.w = 0.73;

    poses.push_back(pose1);
    poses.push_back(pose2);

    navigator->goThroughPoses(poses);
    RCLCPP_INFO(navigator->get_logger(), "Navigating through multiple poses.");
    waitForCompletion(navigator);
    // rclcpp::sleep_for(std::chrono::seconds(2));

    // Cancel Task
    // navigator->cancelTask();
    // RCLCPP_INFO(navigator->get_logger(), "Task canceled.");

    // if (navigator->isTaskComplete() && navigator->getResult() == rclcpp_action::ResultCode::CANCELED) {
    //     RCLCPP_INFO(navigator->get_logger(), "Task was successfully canceled.");
    // } else {
    //     RCLCPP_ERROR(navigator->get_logger(), "Task cancelation failed.");
    // }

    

    if (navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(navigator->get_logger(), "Goal reached successfully, proceeding to next task.");
    } else {
        RCLCPP_ERROR(navigator->get_logger(), "Goal failed!");
        rclcpp::shutdown();
        return 1;
    }

    poses.clear();

    pose1.header.frame_id = "map";
    pose1.pose.position.x = -1.0;
    pose1.pose.position.y = 3.55;
    pose1.pose.orientation.z = -0.02;
    pose1.pose.orientation.w = 0.99;

    pose2.header.frame_id = "map";
    pose2.pose.position.x = -1.0;
    pose2.pose.position.y = 1.3;
    pose2.pose.orientation.z = 0.0;
    pose2.pose.orientation.w = 1.0;

    poses.push_back(pose1);
    poses.push_back(pose2);
    // Check followWaypoints
    navigator->followWaypoints(poses);
    RCLCPP_INFO(navigator->get_logger(), "Following waypoints.");
    waitForCompletion(navigator);

    if (navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(navigator->get_logger(), "Goal reached successfully, proceeding to next task.");
    } else {
        RCLCPP_ERROR(navigator->get_logger(), "Goal failed!");
        rclcpp::shutdown();
        return 1;
    }

    // Check spinning
    navigator->spin(1.57, 2.0);  // spin to 90 degrees during 2 seconds
    RCLCPP_INFO(navigator->get_logger(), "Robot spinning...");

    if (navigator->getResult() == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(navigator->get_logger(), "Goal reached successfully, proceeding to next task.");
    } else {
        RCLCPP_ERROR(navigator->get_logger(), "Goal failed!");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
