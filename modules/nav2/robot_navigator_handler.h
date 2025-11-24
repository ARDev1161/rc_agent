#ifndef ROBOT_NAVIGATOR_HANDLER_H
#define ROBOT_NAVIGATOR_HANDLER_H

#include "robot_navigator.h"
#include "navigation.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_navigator_conversions.h"

using namespace Navigation;

/**
 * @brief Processes an incoming navigation command request.
 *
 * This static function handles the NavCommandRequest by checking which type of command is set
 * (GoToPose, SetInitialPose, GoThroughPoses, FollowWaypoints, Spin, CancelTask, IsTaskComplete, or GetResult)
 * and calls the corresponding method on the RobotNavigator node.
 *
 * @param request The NavCommandRequest protobuf message.
 * @param navigator_node Reference to the RobotNavigator node that will process the command.
 */

static void handleNavCommandRequest(const NavCommandRequest &request, RobotNavigator &navigator_node) {
    RCLCPP_INFO(navigator_node.get_logger(), "Processing NavCommandRequest...");

    if (request.has_go_to_pose()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received GoToPose command.");
        // Вместо ручного копирования используем функцию toPoseStamped
        const auto target_pose = toPoseStamped(request.go_to_pose().pose());
        RCLCPP_INFO(navigator_node.get_logger(),
                    "Navigating to pose (x: %f, y: %f, z: %f)",
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z);
        navigator_node.goToPose(target_pose);

    } else if (request.has_set_initial_pose()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received SetInitialPose command.");
        // Аналогично
        const auto initial_pose = toPoseWithCovarianceStamped(request.set_initial_pose().pose());
        RCLCPP_INFO(navigator_node.get_logger(),
                    "Setting initial pose to (x: %f, y: %f)",
                    initial_pose.pose.pose.position.x,
                    initial_pose.pose.pose.position.y);
        navigator_node.setInitialPose(initial_pose);

    } else if (request.has_go_through_poses()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received GoThroughPoses command.");
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        for (const auto &proto_pose : request.go_through_poses().poses()) {
            poses.push_back(toPoseStamped(proto_pose));
        }
        RCLCPP_INFO(navigator_node.get_logger(), "Going through %lu poses.", poses.size());
        navigator_node.goThroughPoses(poses);

    } else if (request.has_follow_waypoints()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received FollowWaypoints command.");
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        for (const auto &proto_pose : request.follow_waypoints().waypoints()) {
            waypoints.push_back(toPoseStamped(proto_pose));
        }
        RCLCPP_INFO(navigator_node.get_logger(), "Following %lu waypoints.", waypoints.size());
        navigator_node.followWaypoints(waypoints);

    } else if (request.has_spin()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received Spin command.");
        double target_yaw = request.spin().target_yaw();
        double time_allowance = request.spin().time_allowance();
        RCLCPP_INFO(navigator_node.get_logger(), "Spinning to yaw: %f within %f seconds.",
                    target_yaw, time_allowance);
        navigator_node.spin(target_yaw, time_allowance);

    } else if (request.has_cancel_task()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received CancelTask command.");
        navigator_node.cancelTask();
        RCLCPP_INFO(navigator_node.get_logger(), "All tasks have been canceled.");

    } else if (request.has_is_task_complete()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received IsTaskComplete command.");
        bool completed = navigator_node.isTaskComplete();
        RCLCPP_INFO(navigator_node.get_logger(), "Task completion status: %s", completed ? "Completed" : "Not Completed");

    } else if (request.has_get_result()) {
        RCLCPP_INFO(navigator_node.get_logger(), "Received GetResult command.");
        auto result = navigator_node.getResult();
        RCLCPP_INFO(navigator_node.get_logger(), "Navigation task result: %d", static_cast<int>(result));

    } else {
        RCLCPP_WARN(navigator_node.get_logger(), "Unknown navigation command received.");
    }
}

/**
 * @brief Creates a navigation command response based on the current state of the navigator.
 *
 * This static function formulates a NavCommandResponse protobuf message according to the current
 * task status and feedback data from the RobotNavigator node. If feedback is requested, the function
 * fills in the appropriate feedback fields.
 *
 * @param navigator_node Reference to the RobotNavigator node.
 * @param request The original NavCommandRequest protobuf message.
 * @return NavCommandResponse The response to be sent back.
 */
static NavCommandResponse createNavCommandResponse(RobotNavigator &navigator_node, const NavCommandRequest &request) {
    NavCommandResponse response;

    RCLCPP_INFO(navigator_node.get_logger(), "Creating NavCommandResponse...");

    if (navigator_node.isTaskComplete()) {
        auto result = navigator_node.getResult();
        switch (result) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                response.set_status(SUCCESS);
                RCLCPP_INFO(navigator_node.get_logger(), "Navigation succeeded.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                response.set_status(FAILURE);
                RCLCPP_ERROR(navigator_node.get_logger(), "Navigation aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                response.set_status(CANCELED);
                RCLCPP_WARN(navigator_node.get_logger(), "Navigation canceled.");
                break;
            default:
                response.set_status(FAILURE);
                RCLCPP_ERROR(navigator_node.get_logger(), "Unknown result code.");
                break;
        }
    } else {
        response.set_status(IN_PROGRESS);
        RCLCPP_INFO(navigator_node.get_logger(), "Navigation still in progress.");
    }

    // Если пользователь запросил фидбек - заполним.
    if (request.request_feedback()) {
        Response *mut_response = response.mutable_response();
        const std::string &current_task_type = navigator_node.getCurrentTaskType();

        if (current_task_type == "GoToPose") {
            auto *feedback = mut_response->mutable_go_to_pose_feedback();
            feedback->mutable_current_pose()->set_x(navigator_node.getCurrentPose().position.x);
            feedback->mutable_current_pose()->set_y(navigator_node.getCurrentPose().position.y);
            feedback->mutable_current_pose()->set_z(navigator_node.getCurrentPose().position.z);

        } else if (current_task_type == "GoThroughPoses") {
            auto *feedback = mut_response->mutable_go_through_poses_feedback();
            feedback->mutable_current_pose()->set_x(navigator_node.getCurrentPose().position.x);
            feedback->mutable_current_pose()->set_y(navigator_node.getCurrentPose().position.y);
            feedback->mutable_current_pose()->set_z(navigator_node.getCurrentPose().position.z);

        } else if (current_task_type == "FollowWaypoints") {
            auto *feedback = mut_response->mutable_follow_waypoints_feedback();
            feedback->set_current_waypoint(navigator_node.getCurrentWaypointIndex());

        } else if (current_task_type == "Spin") {
            auto *feedback = mut_response->mutable_spin_feedback();
            feedback->set_angular_distance_traveled(navigator_node.getSpinFeedback());
        }
    } else {
        RCLCPP_INFO(navigator_node.get_logger(), "Feedback is not requested, skipping response feedback.");
    }

    return response;
}

#endif // ROBOT_NAVIGATOR_HANDLER_H
