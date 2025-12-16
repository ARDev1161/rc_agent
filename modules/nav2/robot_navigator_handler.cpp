#include "robot_navigator_handler.h"

#include <rclcpp/rclcpp.hpp>
#include "nav_command.h"
#include "robot_navigator.h"
#include "robot_navigator_conversions.h"

using namespace Navigation;

void processNavCommand(const NavCommandRequest &request, RobotNavigator &navigator_node) {
    auto command = NavCommandFactory::create(request);
    if (!command) {
        RCLCPP_WARN(navigator_node.get_logger(), "Unknown navigation command received.");
        return;
    }

    command->execute(navigator_node);
}

NavCommandResponse createNavCommandResponse(RobotNavigator &navigator_node, const NavCommandRequest &request) {
    NavCommandResponse response;

    const auto statusToString = [](CommandStatus status) {
        switch (status) {
            case SUCCESS: return "SUCCESS";
            case FAILURE: return "FAILURE";
            case IN_PROGRESS: return "IN_PROGRESS";
            case CANCELED: return "CANCELED";
            default: return "UNKNOWN";
        }
    };

    static CommandStatus previousStatus = IN_PROGRESS;
    if (navigator_node.isTaskComplete()) {
        auto result = navigator_node.getResult();
        switch (result) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                response.set_status(SUCCESS);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                response.set_status(FAILURE);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                response.set_status(CANCELED);
                break;
            default:
                response.set_status(FAILURE);
                break;
        }
    } else {
        response.set_status(IN_PROGRESS);
    }

    if (response.status() != previousStatus) {
        RCLCPP_INFO(navigator_node.get_logger(),
                    "Navigation status: %s",
                    statusToString(static_cast<CommandStatus>(response.status())));
        previousStatus = static_cast<CommandStatus>(response.status());
    } else if (response.status() == IN_PROGRESS) {
        RCLCPP_DEBUG_THROTTLE(navigator_node.get_logger(),
                              *navigator_node.get_clock(),
                              2000,
                              "Navigation still in progress.");
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
        RCLCPP_DEBUG(navigator_node.get_logger(), "Feedback is not requested, skipping response feedback.");
    }

    return response;
}
