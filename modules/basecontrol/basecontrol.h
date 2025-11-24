#ifndef BASE_CONTROL_HPP
#define BASE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <mutex>
#include "robot.grpc.pb.h"

/**
 * @brief Structure representing base velocities.
 *
 * This structure holds linear and angular velocities.
 */
struct BaseVel {
    double linear;
    double angular;
};

/**
 * @brief Converts a BaseControl protobuf message to a BaseVel structure.
 *
 * This function extracts the linear and angular velocities from the protobuf
 * BaseControl message and returns them in a BaseVel structure.
 *
 * @param base_control_msg A reference to a BaseControl protobuf message.
 * @return BaseVel The converted velocities.
 */
static BaseVel protoToBaseVel(const Base::BaseControl& base_control_msg) {
    return {base_control_msg.linearvelocity(), base_control_msg.angularvelocity()};
}

/**
 * @brief Node class for controlling the base.
 *
 * The BaseControlNode subscribes to odometry messages, publishes velocity commands,
 * and monitors the current control status based on odometry feedback and updates
 * from a BaseControl protobuf message.
 */
class BaseControlNode : public rclcpp::Node {
public:
    /**
     * @brief Enumeration of possible control statuses.
     */
    enum class ControlStatus {
        RUNNING,    ///< The base is moving
        IDLE,       ///< The base is idle
        STOPPED,    ///< The base has stopped
        READY_OK    ///< The base is ready and in good state
    };

    /**
     * @brief Constructor for BaseControlNode.
     *
     * Initializes the node with the name "basecontrol_monitor" and sets up subscriptions,
     * publishers, and a timer to periodically check for updates in the BaseControl protobuf message.
     *
     * @param baseControlMsg Pointer to a BaseControl protobuf message used for monitoring commands.
     */
    explicit BaseControlNode(Base::BaseControl *baseControlMsg);

    /**
     * @brief Publishes a Twist message to a specified topic.
     *
     * Depending on the provided topic name, the function publishes the twist_msg to one of the
     * available publishers.
     *
     * @param twist_msg The Twist message to be published.
     * @param topic The topic name to which the message should be published.
     */
    void publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg,
                                  const std::string& topic);

    /**
     * @brief Retrieves the current control status.
     *
     * The control status is determined based on odometry feedback and internal state.
     *
     * @return ControlStatus The current status of the base control.
     */
    ControlStatus getBaseControlStatus();
    
private:
    class ControlStateMachine;

    /**
     * @brief Callback function for odometry messages.
     *
     * This function is called whenever a new odometry message is received. It updates the
     * actual linear and angular velocities and adjusts the control status accordingly.
     *
     * @param msg Shared pointer to a nav_msgs::msg::Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Checks for updates in the BaseControl protobuf message.
     *
     * This function compares the current velocities from the protobuf message with the
     * last published values. If significant changes are detected, it publishes a new Twist message.
     */
    void checkBaseControlUpdates();

    /// Subscription to odometry messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /// Publisher for Twist messages to the "cmd_vel" topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    /// Publisher for Twist messages to the "cmd_vel_smoothed" topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_smoothed_pub_;
    /// Publisher for Twist messages to the "cmd_vel_unsafety" topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unsafety_pub_;

    /// Timer for periodically calling checkBaseControlUpdates()
    rclcpp::TimerBase::SharedPtr update_timer_;

    double speedCoeff = 0.075;

    double last_cmd_linear_ = 0.0;      ///< Last published linear velocity
    double last_cmd_angular_ = 0.0;     ///< Last published angular velocity
    double actual_linear_vel_ = 0.0;    ///< Actual linear velocity from odometry
    double actual_angular_vel_ = 0.0;   ///< Actual angular velocity from odometry

    /// Pointer to the BaseControl protobuf message used for monitoring control commands.
    Base::BaseControl* baseControlProtoMsg_;

    /// State machine encapsulating status transitions based on odometry
    std::unique_ptr<ControlStateMachine> state_machine_;

    /// Current control status
    ControlStatus current_status_ = ControlStatus::READY_OK;

    /// Mutex for protecting access to the control status
    std::mutex status_mutex_;

    /// Timestamp of the last command received
    rclcpp::Time last_cmd_time_;
};

#endif // BASE_CONTROL_HPP
