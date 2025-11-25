#include "robot_navigator_handler.h"

#include <rclcpp/rclcpp.hpp>
#include "nav_command.h"
#include "robot_navigator.h"
#include "robot_navigator_conversions.h"

using namespace Navigation;

void processNavCommand(const NavCommandRequest &request, RobotNavigator &navigator_node) {
    RCLCPP_INFO(navigator_node.get_logger(), "Processing NavCommandRequest...");

    auto command = NavCommandFactory::create(request);
    if (!command) {
        RCLCPP_WARN(navigator_node.get_logger(), "Unknown navigation command received.");
        return;
    }

    command->execute(navigator_node);
}

NavCommandResponse createNavCommandResponse(RobotNavigator &navigator_node, const NavCommandRequest &request) {
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
