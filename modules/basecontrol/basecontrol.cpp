#include "basecontrol.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cctype>
#include <stdexcept>

using namespace std::chrono_literals;

namespace {

/// Threshold for detecting robot movement from odometry (m/s or rad/s)
constexpr double kVelocityEpsilon = 0.01;
/// Timeout for transitioning to IDLE state when no movement is detected (seconds)
constexpr double kIdleTimeoutSeconds = 3.0;
/// Threshold for detecting changes in commanded velocity (m/s or rad/s)
constexpr double kCommandVelocityEpsilon = 0.001;

} // namespace

class BaseControlNode::ControlStateMachine {
public:
    explicit ControlStateMachine(const rclcpp::Time &start_time)
        : last_cmd_time_(start_time) {}

    BaseControlNode::ControlStatus update(double linear, double angular, const rclcpp::Time &now) {
        return current_->handle(*this, linear, angular, now);
    }

    rclcpp::Time lastCommandTime() const { return last_cmd_time_; }

    template <typename StateT, typename... Args>
    BaseControlNode::ControlStatus transition(Args&&... args) {
        current_ = std::make_unique<StateT>(std::forward<Args>(args)...);
        return current_->state();
    }

    void touchCommandTime(const rclcpp::Time &now) { last_cmd_time_ = now; }

private:
    struct State {
        virtual ~State() = default;
        virtual BaseControlNode::ControlStatus handle(ControlStateMachine &machine, double linear, double angular, const rclcpp::Time &now) = 0;
        virtual BaseControlNode::ControlStatus state() const = 0;
    };

    class ReadyState : public State {
    public:
        BaseControlNode::ControlStatus handle(ControlStateMachine &machine, double linear, double angular, const rclcpp::Time &now) override {
            if (std::abs(linear) > kVelocityEpsilon || std::abs(angular) > kVelocityEpsilon) {
                machine.touchCommandTime(now);
                return machine.transition<RunningState>();
            }
            return state();
        }

        BaseControlNode::ControlStatus state() const override {
            return BaseControlNode::ControlStatus::READY_OK;
        }
    };

    class RunningState : public State {
    public:
        BaseControlNode::ControlStatus handle(ControlStateMachine &machine, double linear, double angular, const rclcpp::Time &now) override {
            if (std::abs(linear) > kVelocityEpsilon || std::abs(angular) > kVelocityEpsilon) {
                machine.touchCommandTime(now);
                return state();
            }
            return machine.transition<StoppedState>();
        }

        BaseControlNode::ControlStatus state() const override {
            return BaseControlNode::ControlStatus::RUNNING;
        }
    };

    class StoppedState : public State {
    public:
        BaseControlNode::ControlStatus handle(ControlStateMachine &machine, double linear, double angular, const rclcpp::Time &now) override {
            if (std::abs(linear) > kVelocityEpsilon || std::abs(angular) > kVelocityEpsilon) {
                machine.touchCommandTime(now);
                return machine.transition<RunningState>();
            }

            const double idle_time = (now - machine.lastCommandTime()).seconds();
            if (idle_time > kIdleTimeoutSeconds) {
                return machine.transition<IdleState>();
            }

            return state();
        }

        BaseControlNode::ControlStatus state() const override {
            return BaseControlNode::ControlStatus::STOPPED;
        }
    };

    class IdleState : public State {
    public:
        BaseControlNode::ControlStatus handle(ControlStateMachine &machine, double linear, double angular, const rclcpp::Time &now) override {
            if (std::abs(linear) > kVelocityEpsilon || std::abs(angular) > kVelocityEpsilon) {
                machine.touchCommandTime(now);
                return machine.transition<RunningState>();
            }
            return state();
        }

        BaseControlNode::ControlStatus state() const override {
            return BaseControlNode::ControlStatus::IDLE;
        }
    };

    std::unique_ptr<State> current_{std::make_unique<ReadyState>()};
    rclcpp::Time last_cmd_time_;
};

BaseControlNode::BaseControlNode(Base::BaseControl *baseControlMsg, std::mutex &grpcMutex,
                                 const rclcpp::NodeOptions &options) :
    Node("basecontrol_monitor", options),
    baseControlProtoMsg_(baseControlMsg),
    grpc_mutex_(grpcMutex),
    state_machine_(std::make_unique<ControlStateMachine>(this->now()))
{
    const auto odom_topic = this->declare_parameter<std::string>("basecontrol.odom_topic", "/odom");
    if (odom_topic.empty()) {
        throw std::invalid_argument("basecontrol.odom_topic must not be empty");
    }
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&BaseControlNode::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to odometry topic %s", odom_topic.c_str());

    cmd_vel_topic_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_topic", "/cmd_vel");
    cmd_vel_msg_type_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_msg_type", "twist");
    cmd_vel_frame_id_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_frame_id", "base_link");
    std::transform(cmd_vel_msg_type_.begin(), cmd_vel_msg_type_.end(), cmd_vel_msg_type_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (cmd_vel_topic_.empty() || cmd_vel_frame_id_.empty()) {
        throw std::invalid_argument("basecontrol cmd_vel topic and frame_id must not be empty");
    }
    if (cmd_vel_msg_type_ != "twist" && cmd_vel_msg_type_ != "twist_stamped") {
        throw std::invalid_argument("Unsupported basecontrol.cmd_vel_msg_type: " + cmd_vel_msg_type_);
    }

    if (cmd_vel_msg_type_ == "twist_stamped") {
        cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    } else {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    // Auxiliary ROS outputs are local ROS parameter bindings, never remote selectors.
    const auto smoothed_topic = this->declare_parameter<std::string>(
        "basecontrol.cmd_vel_smoothed_topic", "/cmd_vel_smoothed");
    const auto nav_topic = this->declare_parameter<std::string>(
        "basecontrol.cmd_vel_nav_topic", "/cmd_vel_nav");
    if (smoothed_topic.empty() || nav_topic.empty()) {
        throw std::invalid_argument("basecontrol auxiliary velocity topics must not be empty");
    }
    cmd_vel_smoothed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(smoothed_topic, 10);
    cmd_vel_unsafety_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(nav_topic, 10);

    // No polling of legacy protobuf velocity: remote motion must enter only
    // through the future authorized v2 TTL/session/lease/deadman path.
    RCLCPP_INFO(this->get_logger(), "BaseControl ROS output: topic=%s type=%s frame_id=%s",
                cmd_vel_topic_.c_str(), cmd_vel_msg_type_.c_str(), cmd_vel_frame_id_.c_str());
}

BaseControlNode::~BaseControlNode() = default;

void BaseControlNode::publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg, const std::string& topic) {
    // Reject malformed velocity commands before publishing on any allowlisted ROS output.
    // This does not grant v2 velocity authority or replace its TTL/lease/watchdog gates.
    const auto &linear = twist_msg.linear;
    const auto &angular = twist_msg.angular;
    if (!std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
        !std::isfinite(angular.x) || !std::isfinite(angular.y) || !std::isfinite(angular.z)) {
        RCLCPP_ERROR(get_logger(), "Rejecting non-finite velocity command");
        return;
    }
    if (topic.compare("cmd_vel") == 0) {
        if (cmd_vel_msg_type_ == "twist_stamped" && cmd_vel_stamped_pub_) {
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = this->now();
            stamped.header.frame_id = cmd_vel_frame_id_;
            stamped.twist = twist_msg;
            cmd_vel_stamped_pub_->publish(stamped);
        } else if (cmd_vel_pub_) {
            cmd_vel_pub_->publish(twist_msg);
        }
    } else if (topic.compare("cmd_vel_smoothed") == 0) {
        cmd_vel_smoothed_pub_->publish(twist_msg);
    } else if (topic.compare("cmd_vel_unsafety") == 0) {
        cmd_vel_unsafety_pub_->publish(twist_msg);
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid topic name: %s", topic.c_str());
    }
}

BaseControlNode::ControlStatus BaseControlNode::getBaseControlStatus() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    RCLCPP_DEBUG(this->get_logger(), "Set BaseControlStatus to: %d", static_cast<int>(current_status_));
    return current_status_;
}

void BaseControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto now = this->now();

    // Keep the odometry sample and state transition atomic with respect to
    // concurrent callbacks in a MultiThreadedExecutor.
    std::lock_guard<std::mutex> lock(status_mutex_);
    // A malformed odometry sample must not drive the state machine or poison
    // the cached motion state. Wait for the next valid sensor sample.
    const auto &twist = msg->twist.twist;
    const double linear = twist.linear.x;
    const double angular = twist.angular.z;
    // Validate the complete six-axis odometry sample even on a planar robot.
    // A malformed unused axis must not advance the cached motion state.
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
        !std::isfinite(twist.linear.z) || !std::isfinite(twist.angular.x) ||
        !std::isfinite(twist.angular.y) || !std::isfinite(twist.angular.z)) {
        RCLCPP_WARN(this->get_logger(), "Ignoring non-finite odometry velocity");
        return;
    }
    actual_linear_vel_ = linear;
    actual_angular_vel_ = angular;
    const ControlStatus innerStatus = state_machine_->update(
        actual_linear_vel_, actual_angular_vel_, now);
    last_cmd_time_ = state_machine_->lastCommandTime();

    {
        if (current_status_ != innerStatus) {
            RCLCPP_DEBUG(this->get_logger(), "Updating status from %d to %d",
                        static_cast<int>(current_status_), static_cast<int>(innerStatus));
            current_status_ = innerStatus;
        }
    }
}

