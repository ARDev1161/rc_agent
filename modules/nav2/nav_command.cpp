#include "nav_command.h"

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include "robot_navigator.h"
#include "robot_navigator_conversions.h"

using Navigation::NavCommandRequest;

namespace {

class GoToPoseCommand : public NavCommand {
public:
  explicit GoToPoseCommand(const NavCommandRequest &request) : command_(request.go_to_pose()) {}

  void execute(RobotNavigator &navigator) override {
    const auto target_pose = toPoseStamped(command_.pose());
    RCLCPP_INFO(navigator.get_logger(),
                "Navigating to pose (x: %f, y: %f, z: %f)",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);
    navigator.goToPose(target_pose);
  }

private:
  Navigation::GoToPose command_;
};

class SetInitialPoseCommand : public NavCommand {
public:
  explicit SetInitialPoseCommand(const NavCommandRequest &request) : command_(request.set_initial_pose()) {}

  void execute(RobotNavigator &navigator) override {
    const auto initial_pose = toPoseWithCovarianceStamped(command_.pose());
    RCLCPP_INFO(navigator.get_logger(),
                "Setting initial pose to (x: %f, y: %f)",
                initial_pose.pose.pose.position.x,
                initial_pose.pose.pose.position.y);
    navigator.setInitialPose(initial_pose);
  }

private:
  Navigation::SetInitialPose command_;
};

class GoThroughPosesCommand : public NavCommand {
public:
  explicit GoThroughPosesCommand(const NavCommandRequest &request) : command_(request.go_through_poses()) {}

  void execute(RobotNavigator &navigator) override {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(command_.poses_size());
    for (const auto &proto_pose : command_.poses()) {
      poses.push_back(toPoseStamped(proto_pose));
    }

    RCLCPP_INFO(navigator.get_logger(), "Going through %zu poses.", poses.size());
    navigator.goThroughPoses(poses);
  }

private:
  Navigation::GoThroughPoses command_;
};

class FollowWaypointsCommand : public NavCommand {
public:
  explicit FollowWaypointsCommand(const NavCommandRequest &request) : command_(request.follow_waypoints()) {}

  void execute(RobotNavigator &navigator) override {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.reserve(command_.waypoints_size());
    for (const auto &proto_pose : command_.waypoints()) {
      waypoints.push_back(toPoseStamped(proto_pose));
    }

    RCLCPP_INFO(navigator.get_logger(), "Following %zu waypoints.", waypoints.size());
    navigator.followWaypoints(waypoints);
  }

private:
  Navigation::FollowWaypoints command_;
};

class SpinCommand : public NavCommand {
public:
  explicit SpinCommand(const NavCommandRequest &request) : command_(request.spin()) {}

  void execute(RobotNavigator &navigator) override {
    const double target_yaw = command_.target_yaw();
    const double time_allowance = command_.time_allowance();
    RCLCPP_INFO(navigator.get_logger(), "Spinning to yaw: %f within %f seconds.",
                target_yaw, time_allowance);
    navigator.spin(target_yaw, time_allowance);
  }

private:
  Navigation::Spin command_;
};

class CancelTaskCommand : public NavCommand {
public:
  void execute(RobotNavigator &navigator) override {
    RCLCPP_INFO(navigator.get_logger(), "Canceling navigation tasks.");
    navigator.cancelTask();
  }
};

class IsTaskCompleteCommand : public NavCommand {
public:
  void execute(RobotNavigator &navigator) override {
    const bool completed = navigator.isTaskComplete();
    RCLCPP_INFO(navigator.get_logger(),
                "Task completion status: %s",
                completed ? "Completed" : "Not Completed");
  }
};

class GetResultCommand : public NavCommand {
public:
  void execute(RobotNavigator &navigator) override {
    auto result = navigator.getResult();
    RCLCPP_INFO(navigator.get_logger(),
                "Navigation task result: %d",
                static_cast<int>(result));
  }
};

using CommandFactory = std::function<std::unique_ptr<NavCommand>(const NavCommandRequest &)>;

const std::unordered_map<NavCommandRequest::CommandCase, CommandFactory> &registry() {
  static const std::unordered_map<NavCommandRequest::CommandCase, CommandFactory> creators{
      {NavCommandRequest::kGoToPose, [](const NavCommandRequest &req) { return std::make_unique<GoToPoseCommand>(req); }},
      {NavCommandRequest::kSetInitialPose, [](const NavCommandRequest &req) { return std::make_unique<SetInitialPoseCommand>(req); }},
      {NavCommandRequest::kGoThroughPoses, [](const NavCommandRequest &req) { return std::make_unique<GoThroughPosesCommand>(req); }},
      {NavCommandRequest::kFollowWaypoints, [](const NavCommandRequest &req) { return std::make_unique<FollowWaypointsCommand>(req); }},
      {NavCommandRequest::kSpin, [](const NavCommandRequest &req) { return std::make_unique<SpinCommand>(req); }},
      {NavCommandRequest::kCancelTask, [](const NavCommandRequest &) { return std::make_unique<CancelTaskCommand>(); }},
      {NavCommandRequest::kIsTaskComplete, [](const NavCommandRequest &) { return std::make_unique<IsTaskCompleteCommand>(); }},
      {NavCommandRequest::kGetResult, [](const NavCommandRequest &) { return std::make_unique<GetResultCommand>(); }},
  };
  return creators;
}

} // namespace

std::unique_ptr<NavCommand> NavCommandFactory::create(const NavCommandRequest &request) {
  const auto it = registry().find(request.command_case());
  if (it == registry().end()) {
    return nullptr;
  }
  return it->second(request);
}
