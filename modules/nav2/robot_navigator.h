#ifndef ROBOT_NAVIGATOR_H
#define ROBOT_NAVIGATOR_H

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <google/protobuf/util/message_differencer.h>
#include "navigation.pb.h"

using namespace Navigation;
using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * @brief Node class for robot navigation using Nav2 action servers.
 *
 * This class manages navigation tasks by sending goals to various Nav2 action servers,
 * processing feedback, and updating protobuf navigation command messages.
 */
class RobotNavigator : public rclcpp::Node {
public:
    /**
     * @brief Constructor for RobotNavigator.
     *
     * Initializes the navigation node with a given name, sets up action clients,
     * publishers, and a timer to periodically check for new navigation commands.
     *
     * @param node_name The name of the node.
     * @param nav_cmd_req Shared pointer to the NavCommandRequest protobuf message.
     * @param nav_cmd_resp Shared pointer to the NavCommandResponse protobuf message.
     * @param grpc_mutex Reference to a mutex used for thread-safe gRPC operations.
     */
    explicit RobotNavigator(const std::string &node_name,
                            std::shared_ptr<NavCommandRequest> nav_cmd_req,
                            std::shared_ptr<NavCommandResponse> nav_cmd_resp,
                            std::mutex &grpc_mutex);

    /**
     * @brief Destructor for RobotNavigator.
     */
    ~RobotNavigator() = default;

    // Methods for executing navigation tasks

    /**
     * @brief Sends a goal to navigate to a specific pose.
     *
     * @param pose The target pose as a PoseStamped message.
     */
    void goToPose(const geometry_msgs::msg::PoseStamped &pose);

    /**
     * @brief Publishes an initial pose for localization.
     *
     * @param pose The initial pose as a PoseWithCovarianceStamped message.
     */
    void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);

    /**
     * @brief Sends a goal to navigate through multiple poses.
     *
     * @param poses A vector of target poses.
     */
    void goThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

    /**
     * @brief Sends a goal to follow a sequence of waypoints.
     *
     * @param waypoints A vector of waypoint poses.
     */
    void followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);

    /**
     * @brief Sends a goal to perform a spin maneuver.
     *
     * @param target_yaw The target yaw angle in degrees.
     * @param time_allowance Time allowance for the spin maneuver.
     */
    void spin(double target_yaw, double time_allowance);

    /**
     * @brief Cancels all active navigation tasks.
     */
    void cancelTask();

    /**
     * @brief Checks if the current navigation task is complete.
     *
     * @return true if the task is complete, false otherwise.
     */
    bool isTaskComplete();

    /**
     * @brief Retrieves the result code of the last navigation task.
     *
     * @return The result code as defined in rclcpp_action::ResultCode.
     */
    rclcpp_action::ResultCode getResult();

    // Methods for retrieving feedback

    /**
     * @brief Gets the type of the current navigation task.
     *
     * @return A string representing the current task type.
     */
    std::string getCurrentTaskType() const;

    /**
     * @brief Gets the current pose of the robot.
     *
     * @return The current pose as a Pose message.
     */
    geometry_msgs::msg::Pose getCurrentPose() const;

    /**
     * @brief Gets the index of the current waypoint (if applicable).
     *
     * @return The current waypoint index.
     */
    int getCurrentWaypointIndex() const;

    /**
     * @brief Gets the feedback value for the spin maneuver.
     *
     * @return The current spin feedback (angular distance traveled).
     */
    double getSpinFeedback() const;

    /**
    * @brief Sends a goal message to the specified action server with attached callbacks for goal response, feedback, and result.
    *
    * This function prepares the given goal message and sets an internal flag indicating
    * that a navigation task is active. It then binds the goal response, feedback,
    * and result callbacks to the provided action client, and asynchronously sends
    * the goal. If \p info_log is non-empty, its content is logged as an informational message.
    *
    * @tparam ActionT The type of the action (e.g., nav2_msgs::action::NavigateToPose).
    * @param goal_msg The goal message of type \c ActionT::Goal to be sent.
    * @param action_client Shared pointer to the action client that will process the goal.
    * @param info_log Optional log message. If non-empty, this message is printed before sending the goal.
    */
    template <typename ActionT>
    void sendGoal(
        const typename ActionT::Goal & goal_msg,
        const typename rclcpp_action::Client<ActionT>::SharedPtr & action_client,
        const std::string & info_log = "");

private:
    // Alias declarations for Nav2 action types
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using Spin = nav2_msgs::action::Spin;

    // Action clients
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;

    // Publisher for setting the initial pose
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    // Template callback methods for handling action server responses
    template <typename T>
    void goalResponseCallback(typename rclcpp_action::ClientGoalHandle<T>::SharedPtr goal_handle);
    template <typename T>
    void feedbackCallback(typename rclcpp_action::ClientGoalHandle<T>::SharedPtr, const std::shared_ptr<const typename T::Feedback> feedback);
    template <typename T>
    void resultCallback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult &result);

    bool task_completed_;                                   ///< Flag indicating if the current task is completed
    rclcpp_action::ResultCode result_code_;                 ///< Result code of the last navigation task
    rclcpp::Time last_feedback_time_;                       ///< Timestamp of the last feedback update

    std::mutex &grpc_mutex_;                                ///< Mutex for thread-safe gRPC operations
    std::mutex task_mutex_;                                 ///< Mutex for protecting task-related data
    std::mutex result_mutex_;                               ///< Mutex for protecting the result code
    std::mutex feedback_mutex_;                             ///< Mutex for protecting feedback callback
    mutable std::mutex feedback_data_mutex_;                ///< Mutex for protecting feedback data

    // Variables for storing feedback data
    std::string current_task_type_;                         ///< Current navigation task type
    geometry_msgs::msg::Pose current_pose_;                 ///< Current robot pose from feedback
    int current_waypoint_index_;                            ///< Index of the current waypoint
    double spin_feedback_;                                  ///< Feedback value for spin action

    // Protobuf messages for navigation commands
    std::shared_ptr<NavCommandRequest> nav_cmd_req_;
    std::shared_ptr<NavCommandResponse> nav_cmd_resp_;

    bool has_active_command_ {false};                       ///< Indicates if a command is currently active
    NavCommandRequest last_command_;                        ///< Last command we started processing

    rclcpp::TimerBase::SharedPtr command_timer_;            ///< Timer for checking new navigation commands

    /**
     * @brief Timer callback function to check for new navigation commands.
     *
     * This function is called periodically to check if a new NavCommandRequest is available,
     * process it safely, and handle it via the corresponding handler.
     */
    void checkNavCommand();
};

#endif // ROBOT_NAVIGATOR_H
