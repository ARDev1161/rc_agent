#include "rclcpp/rclcpp.hpp"
#include "modules/basecontrol/basecontrol.h"
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto controls = std::make_shared<Robot::Controls>();
    std::mutex grpc_mutex;
    auto node = std::make_shared<BaseControlNode>(controls->mutable_basecontrol(), grpc_mutex);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.5;
    twist_msg.angular.z = 0.3;

    RCLCPP_INFO(node->get_logger(), "Publishing test Twist message to cmd_vel...");
    node->publishToBaseControlNode(twist_msg, "cmd_vel");

    rclcpp::Rate rate(1.0);
    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "Waiting for messages...");
        rate.sleep();
    }

    BaseControlNode::ControlStatus status = node->getBaseControlStatus();
    RCLCPP_INFO(node->get_logger(), "Current control status: %d", static_cast<int>(status));

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    node->publishToBaseControlNode(twist_msg, "cmd_vel");

    RCLCPP_INFO(node->get_logger(), "Published stop command, waiting for status update...");

    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "Waiting for status...");
        rate.sleep();
    }

    status = node->getBaseControlStatus();
    RCLCPP_INFO(node->get_logger(), "Updated control status: %d", static_cast<int>(status));

    rclcpp::shutdown();
    return 0;
}
