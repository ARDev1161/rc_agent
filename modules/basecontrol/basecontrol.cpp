#include "basecontrol.h"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace {

constexpr double kVelocityEpsilon = 0.01;
constexpr double kIdleTimeoutSeconds = 3.0;

class ControlStateMachine {
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

} // namespace

BaseControlNode::BaseControlNode(Base::BaseControl *baseControlMsg) :
    Node("basecontrol_monitor"),
    baseControlProtoMsg_(baseControlMsg),
    state_machine_(std::make_unique<ControlStateMachine>(this->now()))
{
    // Subscribe to odometry messages
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&BaseControlNode::odomCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(this->get_logger(), "Subscribed to /odom");

    // Initialize publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_smoothed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed", 10);
    cmd_vel_unsafety_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

    last_cmd_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "BaseControlNode initialized");

    // Create a timer that checks for BaseControl updates every 100 ms
    update_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&BaseControlNode::checkBaseControlUpdates, this));
}

void BaseControlNode::publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg, const std::string& topic) {
    if (topic.compare("cmd_vel") == 0) {
        cmd_vel_pub_->publish(twist_msg);
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
    actual_linear_vel_ = msg->twist.twist.linear.x;
    actual_angular_vel_ = msg->twist.twist.angular.z;

    const auto now = this->now();
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
    // Get the current velocities from the protobuf message
  BaseVel currentVel = protoToBaseVel(*baseControlProtoMsg_);

    // Threshold to determine significant changes
  constexpr double velocity_epsilon = 0.001;
  if (std::abs(currentVel.linear - last_cmd_linear_) > velocity_epsilon ||
      std::abs(currentVel.angular - last_cmd_angular_) > velocity_epsilon)
  {
    // If changes are detected, create a Twist message
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = currentVel.linear * speedCoeff;
    twist_msg.angular.z = -currentVel.angular * speedCoeff;

    // Publish the Twist message to the "cmd_vel" topic
    publishToBaseControlNode(twist_msg, "cmd_vel");

    // Update the last published velocities
    last_cmd_linear_ = currentVel.linear;
    last_cmd_angular_ = currentVel.angular;

    RCLCPP_DEBUG(this->get_logger(), "Published updated cmd_vel: linear=%.3f angular=%.3f",
                currentVel.linear, currentVel.angular);
  }
}
