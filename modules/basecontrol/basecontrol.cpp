#include "basecontrol.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cctype>

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

BaseControlNode::BaseControlNode(Base::BaseControl *baseControlMsg, std::mutex &grpcMutex) :
    Node("basecontrol_monitor"),
    baseControlProtoMsg_(baseControlMsg),
    grpc_mutex_(grpcMutex),
    state_machine_(std::make_unique<ControlStateMachine>(this->now()))
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&BaseControlNode::odomCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(this->get_logger(), "Subscribed to /odom");

    cmd_vel_topic_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_topic", "/cmd_vel");
    cmd_vel_msg_type_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_msg_type", "twist");
    cmd_vel_frame_id_ = this->declare_parameter<std::string>("basecontrol.cmd_vel_frame_id", "base_link");
    input_min_ = this->declare_parameter<double>("basecontrol.input_min", -255.0);
    input_max_ = this->declare_parameter<double>("basecontrol.input_max", 255.0);
    max_linear_speed_ = this->declare_parameter<double>("basecontrol.max_linear_speed", 0.25);
    max_angular_speed_ = this->declare_parameter<double>("basecontrol.max_angular_speed", 0.8);
    input_deadzone_ = this->declare_parameter<double>("basecontrol.input_deadzone", 3.0);

    std::transform(cmd_vel_msg_type_.begin(), cmd_vel_msg_type_.end(), cmd_vel_msg_type_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (cmd_vel_msg_type_ == "twist_stamped") {
        cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    } else {
        cmd_vel_msg_type_ = "twist";
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    cmd_vel_smoothed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed", 10);
    cmd_vel_unsafety_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

    last_cmd_time_ = this->now();
    if (input_max_ <= input_min_) {
        RCLCPP_WARN(this->get_logger(), "Invalid input range [%.3f, %.3f], fallback to [-255,255]", input_min_, input_max_);
        input_min_ = -255.0;
        input_max_ = 255.0;
    }

    RCLCPP_INFO(this->get_logger(), "BaseControl output: topic=%s type=%s frame_id=%s in=[%.1f..%.1f] max_lin=%.3f max_ang=%.3f deadzone=%.1f",
                cmd_vel_topic_.c_str(), cmd_vel_msg_type_.c_str(), cmd_vel_frame_id_.c_str(),
                input_min_, input_max_, max_linear_speed_, max_angular_speed_, input_deadzone_);

    update_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&BaseControlNode::checkBaseControlUpdates, this));
}

BaseControlNode::~BaseControlNode() = default;

void BaseControlNode::publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg, const std::string& topic) {
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

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        actual_linear_vel_ = msg->twist.twist.linear.x;
        actual_angular_vel_ = msg->twist.twist.angular.z;
    }

    ControlStatus innerStatus = state_machine_->update(actual_linear_vel_, actual_angular_vel_, now);
    last_cmd_time_ = state_machine_->lastCommandTime();

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (current_status_ != innerStatus) {
            RCLCPP_DEBUG(this->get_logger(), "Updating status from %d to %d",
                        static_cast<int>(current_status_), static_cast<int>(innerStatus));
            current_status_ = innerStatus;
        }
    }
}

void BaseControlNode::checkBaseControlUpdates() {
    BaseVel currentVel;
    {
      std::lock_guard<std::mutex> lock(grpc_mutex_);
      currentVel = protoToBaseVel(*baseControlProtoMsg_);
    }

    if (std::abs(currentVel.linear - last_cmd_linear_) > kCommandVelocityEpsilon ||
        std::abs(currentVel.angular - last_cmd_angular_) > kCommandVelocityEpsilon)
    {
        auto apply_deadzone = [this](double value) {
            return (std::abs(value) < input_deadzone_) ? 0.0 : value;
        };

        const double range = input_max_ - input_min_;
        const double linear_norm = std::clamp((apply_deadzone(currentVel.linear) - input_min_) / range * 2.0 - 1.0, -1.0, 1.0);
        const double angular_norm = std::clamp((apply_deadzone(currentVel.angular) - input_min_) / range * 2.0 - 1.0, -1.0, 1.0);

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_norm * max_linear_speed_;
        twist_msg.angular.z = -angular_norm * max_angular_speed_;

        publishToBaseControlNode(twist_msg, "cmd_vel");

        last_cmd_linear_ = currentVel.linear;
        last_cmd_angular_ = currentVel.angular;

        RCLCPP_DEBUG(this->get_logger(), "Published updated cmd_vel: linear=%.3f angular=%.3f",
                    currentVel.linear, currentVel.angular);
    }
}
