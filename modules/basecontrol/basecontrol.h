#ifndef BASE_CONTROL_HPP
#define BASE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <mutex>
#include "robot.grpc.pb.h"

struct BaseVel {
    double linear;
    double angular;
};

static BaseVel protoToBaseVel(const Base::BaseControl& base_control_msg) {
    return {base_control_msg.linearvelocity(), base_control_msg.angularvelocity()};
}

class BaseControlNode : public rclcpp::Node {
public:
    enum class ControlStatus {
        RUNNING,
        IDLE,
        STOPPED,
        READY_OK
    };

    explicit BaseControlNode(Base::BaseControl *baseControlMsg, std::mutex &grpcMutex);
    ~BaseControlNode();

    void publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg,
                                  const std::string& topic);

    ControlStatus getBaseControlStatus();

private:
    class ControlStateMachine;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void checkBaseControlUpdates();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_smoothed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unsafety_pub_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    std::string cmd_vel_topic_;
    std::string cmd_vel_msg_type_;
    std::string cmd_vel_frame_id_;

    double input_min_ = -255.0;
    double input_max_ = 255.0;
    double max_linear_speed_ = 0.25;
    double max_angular_speed_ = 0.8;
    double input_deadzone_ = 3.0;

    double last_cmd_linear_ = 0.0;
    double last_cmd_angular_ = 0.0;
    double actual_linear_vel_ = 0.0;
    double actual_angular_vel_ = 0.0;

    Base::BaseControl* baseControlProtoMsg_;
    std::mutex &grpc_mutex_;

    std::unique_ptr<ControlStateMachine> state_machine_;

    ControlStatus current_status_ = ControlStatus::READY_OK;
    std::mutex status_mutex_;

    rclcpp::Time last_cmd_time_;
};

#endif // BASE_CONTROL_HPP
