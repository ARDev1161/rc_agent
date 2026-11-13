#ifndef BASE_CONTROL_HPP
#define BASE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <mutex>
#include "robot.grpc.pb.h"

class BaseControlNode : public rclcpp::Node {
public:
    enum class ControlStatus {
        RUNNING,
        IDLE,
        STOPPED,
        READY_OK
    };

    explicit BaseControlNode(Base::BaseControl *baseControlMsg, std::mutex &grpcMutex,
                             const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~BaseControlNode();

    void publishToBaseControlNode(const geometry_msgs::msg::Twist& twist_msg,
                                  const std::string& topic);

    ControlStatus getBaseControlStatus();

private:
    class ControlStateMachine;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_smoothed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unsafety_pub_;


    std::string cmd_vel_topic_;
    std::string cmd_vel_msg_type_;
    std::string cmd_vel_frame_id_;

    double actual_linear_vel_ = 0.0;
    double actual_angular_vel_ = 0.0;

    Base::BaseControl* baseControlProtoMsg_;
    std::mutex &grpc_mutex_;

    std::unique_ptr<ControlStateMachine> state_machine_;

    ControlStatus current_status_ = ControlStatus::READY_OK;
    std::mutex status_mutex_;

};

#endif // BASE_CONTROL_HPP
