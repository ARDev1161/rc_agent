#ifndef ROBOT_NAVIGATOR_CONVERSIONS_H
#define ROBOT_NAVIGATOR_CONVERSIONS_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "navigation.pb.h"

// Пример: преобразование Navigation::Pose -> geometry_msgs::msg::PoseStamped
inline geometry_msgs::msg::PoseStamped
toPoseStamped(const Navigation::Pose &proto_pose,
              const std::string &frame_id = "map")
{
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header.frame_id = frame_id;

    ros_pose.pose.position.x = proto_pose.x();
    ros_pose.pose.position.y = proto_pose.y();
    ros_pose.pose.position.z = proto_pose.z();

    ros_pose.pose.orientation.w = proto_pose.orientation_w();
    ros_pose.pose.orientation.x = proto_pose.orientation_x();
    ros_pose.pose.orientation.y = proto_pose.orientation_y();
    ros_pose.pose.orientation.z = proto_pose.orientation_z();

    return ros_pose;
}

// Пример: преобразование Navigation::PoseWithCovariance -> geometry_msgs::msg::PoseWithCovarianceStamped
inline geometry_msgs::msg::PoseWithCovarianceStamped
toPoseWithCovarianceStamped(const Navigation::PoseWithCovariance &proto_pose_cov,
                            const std::string &frame_id = "map")
{
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov;
    ros_pose_cov.header.frame_id = frame_id;

    // Заполняем pose
    ros_pose_cov.pose.pose.position.x  = proto_pose_cov.pose().x();
    ros_pose_cov.pose.pose.position.y  = proto_pose_cov.pose().y();
    ros_pose_cov.pose.pose.position.z  = proto_pose_cov.pose().z();
    ros_pose_cov.pose.pose.orientation.w = proto_pose_cov.pose().orientation_w();
    ros_pose_cov.pose.pose.orientation.x = proto_pose_cov.pose().orientation_x();
    ros_pose_cov.pose.pose.orientation.y = proto_pose_cov.pose().orientation_y();
    ros_pose_cov.pose.pose.orientation.z = proto_pose_cov.pose().orientation_z();

    // При необходимости заполнить covariance:
    // если в Navigation::PoseWithCovariance есть массив ковариации, нужно его скопировать
    // например:
    // for (int i = 0; i < 36; ++i) {
    //   ros_pose_cov.pose.covariance[i] = proto_pose_cov.covariance(i);
    // }

    return ros_pose_cov;
}

#endif // ROBOT_NAVIGATOR_CONVERSIONS_H
