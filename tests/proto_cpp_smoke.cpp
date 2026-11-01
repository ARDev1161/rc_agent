#include <cassert>
#include <string>

#include "base.pb.h"
#include "map.pb.h"
#include "navigation.pb.h"
#include "rc_api_v2.pb.h"
#include "robot.pb.h"
#include "video.pb.h"

int main() {
  Robot::Controls legacy;
  legacy.mutable_basecontrol()->set_linearvelocity(48.0);
  legacy.mutable_basecontrol()->set_angularvelocity(-17.0);
  legacy.mutable_navcontrol()->mutable_go_to_pose()->mutable_pose()->set_x(2.0);
  legacy.mutable_video_config()->set_use_widget_size(true);

  std::string wire;
  assert(legacy.SerializeToString(&wire));
  Robot::Controls controls;
  assert(controls.ParseFromString(wire));
  assert(controls.basecontrol().linearvelocity() == 48.0);
  assert(controls.navcontrol().has_go_to_pose());
  assert(controls.video_config().use_widget_size());

  Robot::Sensors sensors;
  sensors.mutable_robot_info()->set_robot_id("robot-001");
  sensors.mutable_robot_info()->set_api_major(2);
  auto *capability = sensors.mutable_robot_info()->add_capabilities();
  capability->set_id("manual.velocity");
  capability->add_velocity_axes("linear.x");
  capability->add_velocity_axes("angular.z");
  sensors.mutable_navcontrolstatus()->set_status(Navigation::IN_PROGRESS);
  sensors.mutable_video_stream()->set_pipeline("udpsrc port=1234");

  assert(sensors.SerializeToString(&wire));
  Robot::Sensors restored;
  assert(restored.ParseFromString(wire));
  assert(restored.has_robot_info());
  assert(restored.robot_info().robot_id() == "robot-001");
  assert(restored.robot_info().capabilities(0).velocity_axes_size() == 2);
  assert(restored.navcontrolstatus().status() == Navigation::IN_PROGRESS);
  assert(restored.video_stream().pipeline() == "udpsrc port=1234");

  rc::api::v2::ChatMessage chat;
  chat.set_text("Are you there?");
  assert(chat.has_text());
  chat.set_audio(std::string(8, '\0'));
  assert(chat.has_audio() && !chat.has_text());

  rc::api::v2::TaskStatus stage;
  stage.set_task_id("task1");
  stage.set_parent_id("task0");
  stage.set_state(rc::api::v2::TaskStatus::ACCEPTED);
  assert(stage.state() != rc::api::v2::TaskStatus::SUCCEEDED);

  return 0;
}
