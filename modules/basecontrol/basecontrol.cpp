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
    // Distinct ROS output names are a safety boundary: an auxiliary publisher
    // must never alias the guarded cmd_vel output and bypass its watchdog.
    // ROS resolves "cmd_vel" and "/cmd_vel" to the same topic in the root
    // namespace. Compare their canonical names, not their config spelling,
    // or an auxiliary publisher could alias the lease-guarded output.
    const auto canonical_topic = [](const std::string &topic) {
        return topic.front() == '/' ? topic : "/" + topic;
    };
    if (canonical_topic(smoothed_topic) == canonical_topic(cmd_vel_topic_) ||
        canonical_topic(nav_topic) == canonical_topic(cmd_vel_topic_) ||
        canonical_topic(smoothed_topic) == canonical_topic(nav_topic)) {
        throw std::invalid_argument("basecontrol velocity output topics must be distinct");
    }
    cmd_vel_smoothed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(smoothed_topic, 10);
    cmd_vel_unsafety_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(nav_topic, 10);

    // Monotonic watchdog remains active independently of network callbacks.
    // The gate is fail-closed: without an authenticated lease, no motion is sent.
    velocity_watchdog_timer_ = this->create_wall_timer(
        50ms, std::bind(&BaseControlNode::checkVelocityWatchdog, this));

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
        // The public generic publisher has no authenticated v2 session/lease.
        // It may issue an emergency zero, but must never bypass VelocityGate
        // to publish motion. A future authorized path must be separate.
        const bool nonzero = linear.x != 0.0 || linear.y != 0.0 || linear.z != 0.0 ||
                             angular.x != 0.0 || angular.y != 0.0 || angular.z != 0.0;
        if (nonzero) {
            RCLCPP_ERROR(get_logger(), "Rejecting direct cmd_vel motion without v2 lease");
            return;
        }
        // Emergency zero and leased writes must not race on the DDS publisher.
        std::lock_guard<std::mutex> publish_lock(velocity_publish_mutex_);
        velocity_gate_.revoke();
        bool stop_published = false;
        try {
        if (cmd_vel_msg_type_ == "twist_stamped" && cmd_vel_stamped_pub_) {
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = this->now();
            stamped.header.frame_id = cmd_vel_frame_id_;
            stamped.twist = twist_msg;
            cmd_vel_stamped_pub_->publish(stamped);
            stop_published = true;
        } else if (cmd_vel_pub_) {
            cmd_vel_pub_->publish(twist_msg);
            stop_published = true;
        }
        // The zero was successfully handed to DDS; clear the stop latch.
        // Keep it latched if publishing throws or no publisher is available.
        if (stop_published) velocity_gate_.acknowledgeStopPublished();
        } catch (const std::exception& e) {
            // Keep the stop latched; the watchdog will retry on its next tick.
            RCLCPP_ERROR(get_logger(), "Emergency zero DDS publish failed: %s", e.what());
        }
    } else if (topic.compare("cmd_vel_smoothed") == 0) {
        cmd_vel_smoothed_pub_->publish(twist_msg);
    } else if (topic.compare("cmd_vel_unsafety") == 0) {
        cmd_vel_unsafety_pub_->publish(twist_msg);
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid topic name: %s", topic.c_str());
    }
}

bool BaseControlNode::publishLeasedVelocity(const rc::api::v2::Velocity& velocity,
                                            const std::string& session,
                                            const std::string& lease, bool deadman) {
    // Admission checks TTL, monotonic sequence, finite SI values and lease identity.
    // No scaling or legacy Controls path is involved.
    // Serialize admission as well as publication: concurrent commands must
    // not publish out of sequence or race an emergency stop.
    std::lock_guard<std::mutex> publish_lock(velocity_publish_mutex_);
    const auto now = rc::api::v2::VelocityGate::Clock::now();
    if (velocity.frame_id() != cmd_vel_frame_id_) {
        velocity_gate_.revoke();
        // The 50 ms watchdog publishes the latched zero after this lock releases.
        return false;
    }
    if (!velocity_gate_.admit(velocity, session, lease, deadman, now)) {
        // The 50 ms watchdog publishes the latched zero after this lock releases.
        return false;
    }

    // A watchdog stop can race with admission. Hold the DDS publication lock
    // and recheck the monotonic deadline before emitting any nonzero motion.
    if (velocity_gate_.expired(rc::api::v2::VelocityGate::Clock::now())) {
        velocity_gate_.revoke();
        return false; // Timer publishes the latched zero without lock recursion.
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.linear().x();
    twist.linear.y = velocity.linear().y();
    twist.linear.z = velocity.linear().z();
    twist.angular.x = velocity.angular().x();
    twist.angular.y = velocity.angular().y();
    twist.angular.z = velocity.angular().z();
    try {
        if (cmd_vel_msg_type_ == "twist_stamped") {
            if (!cmd_vel_stamped_pub_) {
                velocity_gate_.revoke();
                return false;
            }
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = this->now();
            stamped.header.frame_id = cmd_vel_frame_id_;
            stamped.twist = twist;
            cmd_vel_stamped_pub_->publish(stamped);
        } else {
            if (!cmd_vel_pub_) {
                velocity_gate_.revoke();
                return false;
            }
            cmd_vel_pub_->publish(twist);
        }
        return true;
    } catch (const std::exception& e) {
        velocity_gate_.revoke();
        RCLCPP_ERROR(get_logger(), "Leased velocity DDS publish failed: %s", e.what());
        return false;
    }
}

void BaseControlNode::checkVelocityWatchdog() {
    std::lock_guard<std::mutex> publish_lock(velocity_publish_mutex_);
    if (!velocity_gate_.stopPending(rc::api::v2::VelocityGate::Clock::now())) return;
    // Only the configured cmd_vel publisher is used. A failed publish must
    // leave the gate latched so the next timer tick retries the stop.
    try {
        geometry_msgs::msg::Twist zero;
        if (cmd_vel_msg_type_ == "twist_stamped") {
            if (!cmd_vel_stamped_pub_) return;
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = this->now();
            stamped.header.frame_id = cmd_vel_frame_id_;
            stamped.twist = zero;
            cmd_vel_stamped_pub_->publish(stamped);
        } else {
            if (!cmd_vel_pub_) return;
            cmd_vel_pub_->publish(zero);
        }
        velocity_gate_.acknowledgeStopPublished();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Velocity watchdog stop publish failed: %s", e.what());
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

