#include "robot_navigator_handler.h"
#include "robot_navigator.h"


RobotNavigator::RobotNavigator(const std::string &node_name,
                               std::shared_ptr<NavCommandRequest> nav_cmd_req,
                               std::shared_ptr<NavCommandResponse> nav_cmd_resp,
                               std::mutex &grpc_mutex)
  : rclcpp::Node(node_name),
    last_feedback_time_(this->now()),
    task_completed_(false),
    result_code_(rclcpp_action::ResultCode::UNKNOWN),
    nav_cmd_req_(nav_cmd_req),
    nav_cmd_resp_(nav_cmd_resp),
    grpc_mutex_(grpc_mutex)
{
    // Initialize action clients
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    navigate_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
    follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");

    // Initialize publisher for initial pose
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    // Check availability of action servers
    if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available!");
        rclcpp::shutdown();
        return;
    }
    if (!navigate_through_poses_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "navigate_through_poses action server not available!");
        rclcpp::shutdown();
        return;
    }
    if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "follow_waypoints action server not available!");
        rclcpp::shutdown();
        return;
    }
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "spin action server not available!");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(get_logger(), "Connected to action server.");

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

    // If no command is set, exit
    if (nav_cmd_req_->command_case() == NavCommandRequest::COMMAND_NOT_SET) {
        return;
    }

    // Create a safe copy of the command to avoid concurrent access issues
    NavCommandRequest safeCopy;
    safeCopy.CopyFrom(*nav_cmd_req_);

    // Static variable to track the previous command to avoid processing duplicates
    static std::string prev = "";
    std::string cur = safeCopy.DebugString();

    if (prev != cur) {
        prev = cur;
        // Process the navigation command. The function handleNavCommandRequest must be defined in the handler
        handleNavCommandRequest(safeCopy, *this);

        // Creating a response if nav_cmd_resp_ is valid and createNavCommandResponse is defined
        if (nav_cmd_resp_) {
            *nav_cmd_resp_ = createNavCommandResponse(*this, safeCopy);
        }
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
    // Cancel for navigate_to_pose
    auto future_cancel_pose = navigate_to_pose_client_->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel_pose) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel navigate_to_pose tasks.");
    }
    // Cancel for navigate_through_poses
    auto future_cancel_through = navigate_through_poses_client_->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel_through) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel navigate_through_poses tasks.");
    }
    // Cancel for follow_waypoints
    auto future_cancel_waypoints = follow_waypoints_client_->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel_waypoints) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel follow_waypoints tasks.");
    }
    // Cancel for spin
    auto future_cancel_spin = spin_client_->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel_spin) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel spin tasks.");
    }
    RCLCPP_INFO(this->get_logger(), "All navigation tasks have been canceled.");
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
        std::bind(&RobotNavigator::goalResponseCallback<ActionT>, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&RobotNavigator::feedbackCallback<ActionT>, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&RobotNavigator::resultCallback<ActionT>, this, std::placeholders::_1);

    // Асинхронно отправляем goal
    action_client->async_send_goal(goal_msg, send_goal_options);
}

template <typename T>
void RobotNavigator::goalResponseCallback(typename rclcpp_action::ClientGoalHandle<T>::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Task was rejected by server.");
    } else {
        RCLCPP_INFO(get_logger(), "Task accepted by server.");
    }
}

template <typename T>
void RobotNavigator::feedbackCallback(
    typename rclcpp_action::ClientGoalHandle<T>::SharedPtr, 
    const std::shared_ptr<const typename T::Feedback> feedback)
{
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
void RobotNavigator::resultCallback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult &result) {
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
