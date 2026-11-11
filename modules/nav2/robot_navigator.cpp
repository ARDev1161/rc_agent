#include "robot_navigator_handler.h"
#include "robot_navigator.h"
#include <stdexcept>


RobotNavigator::RobotNavigator(const std::string &node_name,
                               std::shared_ptr<NavCommandRequest> nav_cmd_req,
                               std::shared_ptr<NavCommandResponse> nav_cmd_resp,
                               std::mutex &grpc_mutex,
                               const rclcpp::NodeOptions &options)
  : rclcpp::Node(node_name, options),
    last_feedback_time_(this->now()),
    task_completed_(false),
    result_code_(rclcpp_action::ResultCode::UNKNOWN),
    nav_cmd_req_(nav_cmd_req),
    nav_cmd_resp_(nav_cmd_resp),
    grpc_mutex_(grpc_mutex)
{
    // Only locally configured endpoint names are accepted; defaults retain dev wiring.
    const auto navigate_to_pose = declare_parameter<std::string>(
        "nav2.navigate_to_pose_action", "navigate_to_pose");
    const auto navigate_through_poses = declare_parameter<std::string>(
        "nav2.navigate_through_poses_action", "navigate_through_poses");
    const auto follow_waypoints = declare_parameter<std::string>(
        "nav2.follow_waypoints_action", "follow_waypoints");
    const auto spin = declare_parameter<std::string>("nav2.spin_action", "spin");
    const auto initial_pose = declare_parameter<std::string>(
        "nav2.initial_pose_topic", "initialpose");
    if (navigate_to_pose.empty() || navigate_through_poses.empty() ||
        follow_waypoints.empty() || spin.empty() || initial_pose.empty()) {
        throw std::invalid_argument("Nav2 action and initial pose endpoints must not be empty");
    }

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose);
    navigate_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
        this, navigate_through_poses);
    follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, follow_waypoints);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin);
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose, 10);

    // Nav2 may start after rc_agent. Do not block startup for 40 seconds or
    // shut down the entire ROS context when one optional action is absent.
    // Client availability is checked when a command is actually submitted.
    RCLCPP_INFO(get_logger(), "Nav2 clients configured; action servers may connect later.");

    // Create a timer to check for new navigation commands every 100 ms
    command_timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotNavigator::checkNavCommand, this));
}

void RobotNavigator::checkNavCommand() {
    std::lock_guard<std::mutex> lock(grpc_mutex_);

    if (!nav_cmd_req_ || !nav_cmd_req_->IsInitialized()) {
        RCLCPP_ERROR(this->get_logger(), "nav_cmd_req_ is null or uninitialized!");
        return;
    }
    if (!nav_cmd_resp_) {
        RCLCPP_ERROR(this->get_logger(), "nav_cmd_resp_ is null!");
        return;
    }

    // If no command is set, exit
    if (nav_cmd_req_->command_case() == NavCommandRequest::COMMAND_NOT_SET) {
        if (has_active_command_) {
            *nav_cmd_resp_ = createNavCommandResponse(*this, last_command_);
        }
        return;
    }

    // Create a safe copy of the command to avoid concurrent access issues
    NavCommandRequest safeCopy;
    safeCopy.CopyFrom(*nav_cmd_req_);

    // Process a new command if it's different or no active command is running
    if (!has_active_command_ || !google::protobuf::util::MessageDifferencer::Equals(safeCopy, last_command_)) {
        last_command_ = safeCopy;
        has_active_command_ = true;
        processNavCommand(last_command_, *this);
    }

    // Always publish current status/feedback while command is active
    *nav_cmd_resp_ = createNavCommandResponse(*this, last_command_);

    // If task finished, clear active flag and request to allow the same command to be re-sent (patrol)
    if (isTaskComplete()) {
        has_active_command_ = false;
        nav_cmd_req_->Clear();
    }
}

void RobotNavigator::goToPose(const geometry_msgs::msg::PoseStamped &pose) {
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = pose;

    std::string info = "Sending goal to x: " + std::to_string(pose.pose.position.x) +
                       ", y: " + std::to_string(pose.pose.position.y);

    sendGoal<NavigateToPose>(goal_msg, navigate_to_pose_client_, info);
}

void RobotNavigator::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
    initial_pose_pub_->publish(pose);
    RCLCPP_INFO(this->get_logger(),
                "Initial pose set at x: %.2f, y: %.2f",
                pose.pose.pose.position.x,
                pose.pose.pose.position.y);
}

void RobotNavigator::goThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
    NavigateThroughPoses::Goal goal_msg;
    goal_msg.poses = poses;

    std::string info = "Sending NavigateThroughPoses goal with " +
                       std::to_string(poses.size()) + " poses";

    sendGoal<NavigateThroughPoses>(goal_msg, navigate_through_poses_client_, info);
}

void RobotNavigator::followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) {
    FollowWaypoints::Goal goal_msg;
    goal_msg.poses = waypoints;

    std::string info = "Sending FollowWaypoints goal with " +
                       std::to_string(waypoints.size()) + " waypoints";

    sendGoal<FollowWaypoints>(goal_msg, follow_waypoints_client_, info);
}

void RobotNavigator::spin(double target_yaw, double time_allowance) {
    Spin::Goal goal_msg;
    goal_msg.target_yaw = target_yaw;
    goal_msg.time_allowance.sec = static_cast<int32_t>(time_allowance);
    goal_msg.time_allowance.nanosec = 0;

    std::string info = "Sending Spin goal with target yaw " + std::to_string(target_yaw);

    sendGoal<Spin>(goal_msg, spin_client_, info);
}

void RobotNavigator::cancelTask() {
    // A completed action has no current goal to cancel. In particular, a
    // second cancel after SUCCEEDED/CANCELED must not reach the backend.
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (task_completed_) return;
    }
    // A cancellation request is not a terminal result. Nav2 may reject it or
    // complete the goal before cancellation is processed. Only resultCallback()
    // may publish CANCELED/SUCCEEDED/ABORTED and mark the task complete.
    std::function<void()> cancel;
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        if (active_goal_generation_ != goal_generation_.load()) return;
        // Repeated operator requests must not send duplicate cancel RPCs.
        if (cancel_requested_) return;
        cancel_requested_ = true;
        cancel = cancel_active_goal_;
    }
    // The goal may not have been accepted yet. goalResponseCallback will
    // dispatch the pending cancellation once the current goal handle arrives.
    if (cancel) cancel();
    RCLCPP_INFO(this->get_logger(), "Requested cancellation of navigation goals; awaiting Nav2 result.");
}

bool RobotNavigator::isTaskComplete() {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return task_completed_ ;

}

rclcpp_action::ResultCode RobotNavigator::getResult() {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return result_code_;    
}

std::string RobotNavigator::getCurrentTaskType() const {
  std::lock_guard<std::mutex> lock(feedback_data_mutex_);
  return current_task_type_;
}

geometry_msgs::msg::Pose RobotNavigator::getCurrentPose() const {
  std::lock_guard<std::mutex> lock(feedback_data_mutex_);
  return current_pose_;
}

int RobotNavigator::getCurrentWaypointIndex() const {
  std::lock_guard<std::mutex> lock(feedback_data_mutex_);
  return current_waypoint_index_;
}

double RobotNavigator::getSpinFeedback() const {
  std::lock_guard<std::mutex> lock(feedback_data_mutex_);
  return spin_feedback_;
}

template <typename ActionT>
void RobotNavigator::sendGoal(
    const typename ActionT::Goal & goal_msg,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & action_client,
    const std::string & info_log)
{
    const auto generation = goal_generation_.fetch_add(1) + 1;
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_goal_generation_ = generation;
        cancel_requested_ = false;
        cancel_active_goal_ = {};
    }
    // A new goal must never expose the previous goal's terminal result while
    // Nav2 has not yet accepted or completed this request.
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    }
    // Ставим флаг, что задача ещё не завершена
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_completed_ = false;
    }

    // Выводим в лог, если передали info_log
    if (!info_log.empty()) {
        RCLCPP_INFO(get_logger(), "%s", info_log.c_str());
    }

    // Настраиваем колбэки (goal_response, feedback, result)
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this, generation, action_client](auto handle) {
            std::function<void()> cancel;
            {
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                if (generation != active_goal_generation_) return;
                if (handle) {
                    cancel_active_goal_ = [this, generation, action_client, handle]() {
                        // A replacement may occur after the callback copies this
                        // closure but before it dispatches the cancellation.
                        // Never cancel a superseded goal in that window.
                        std::lock_guard<std::mutex> lock(active_goal_mutex_);
                        if (generation != active_goal_generation_) return;
                        action_client->async_cancel_goal(handle);
                    };
                    if (cancel_requested_) cancel = cancel_active_goal_;
                }
            }
            if (cancel) cancel();
            goalResponseCallback<ActionT>(handle, generation);
        };
    send_goal_options.feedback_callback =
        [this, generation](auto handle, auto feedback) {
            feedbackCallback<ActionT>(handle, feedback, generation);
        };
    send_goal_options.result_callback =
        [this, generation](const auto &result) { resultCallback<ActionT>(result, generation); };

    // Fail this command without shutting down unrelated ROS2 nodes.
    if (!action_client->action_server_is_ready()) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_code_ = rclcpp_action::ResultCode::UNKNOWN;
        }
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            task_completed_ = true;
        }
        RCLCPP_ERROR(get_logger(), "Nav2 action server is not ready; goal was not sent.");
        return;
    }

    // Асинхронно отправляем goal
    action_client->async_send_goal(goal_msg, send_goal_options);
}

template <typename T>
void RobotNavigator::goalResponseCallback(typename rclcpp_action::ClientGoalHandle<T>::SharedPtr goal_handle, std::uint64_t generation) {
    if (generation != goal_generation_.load()) return;
    if (!goal_handle) {
        // Rejected goals have no result callback. Complete the request as an
        // unsuccessful outcome rather than leaving the workstation waiting.
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_code_ = rclcpp_action::ResultCode::UNKNOWN;
        }
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            task_completed_ = true;
        }
        RCLCPP_ERROR(get_logger(), "Task was rejected by server.");
    } else {
        RCLCPP_INFO(get_logger(), "Task accepted by server.");
    }
}

template <typename T>
void RobotNavigator::feedbackCallback(
    typename rclcpp_action::ClientGoalHandle<T>::SharedPtr, 
    const std::shared_ptr<const typename T::Feedback> feedback, std::uint64_t generation)
{
    if (generation != goal_generation_.load()) return;
    auto now = this->now();

    if ((now - last_feedback_time_).seconds() > 0.5) { // Update feedback every half of second
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_feedback_time_ = now;
        }
        
        // Update feedback data with mutex protection
        std::lock_guard<std::mutex> lock(feedback_data_mutex_);
        if constexpr (std::is_same_v<T, nav2_msgs::action::NavigateToPose>) {
            current_task_type_ = "GoToPose";
            current_pose_ = feedback->current_pose.pose;

            RCLCPP_INFO(get_logger(), "[ToPoses] Feedback: currently at x: %.2f, y: %.2f",
                        feedback->current_pose.pose.position.x, 
                        feedback->current_pose.pose.position.y);

        } else if constexpr (std::is_same_v<T, nav2_msgs::action::NavigateThroughPoses>) {
            current_task_type_ = "GoThroughPoses";
            current_pose_ = feedback->current_pose.pose;

            RCLCPP_INFO(get_logger(), "[ThroughPoses] Feedback: currently at x: %.2f, y: %.2f",
                        feedback->current_pose.pose.position.x, 
                        feedback->current_pose.pose.position.y);

        } else if constexpr (std::is_same_v<T, nav2_msgs::action::FollowWaypoints>) {
            current_task_type_ = "FollowWaypoints";
            current_waypoint_index_ = static_cast<int>(feedback->current_waypoint);

            RCLCPP_INFO(get_logger(), "[FollowWayPoint] Feedback: following waypoint %u",
                        feedback->current_waypoint);

        } else if constexpr (std::is_same_v<T, nav2_msgs::action::Spin>) {
            current_task_type_ = "Spin";
            spin_feedback_ = feedback->angular_distance_traveled;

            RCLCPP_INFO(get_logger(), "[Spin] Feedback: current yaw %.2f degrees",
                        feedback->angular_distance_traveled);

        } else {
            RCLCPP_WARN(get_logger(), "Unknown feedback type received");
        }
    }
}

template <typename T>
void RobotNavigator::resultCallback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult &result, std::uint64_t generation) {
    if (generation != goal_generation_.load()) return;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_code_ = result.code;
    }
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_completed_ = true;
    }
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Task succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Task was aborted!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Task was canceled!");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
            break;
    }
}
