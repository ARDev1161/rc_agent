"""Regression guard for Nav2 goal lifecycle transitions (source contract only).

ROS2 action-server runtime acceptance remains a separate P2 gate.
"""
from pathlib import Path
import re
import unittest

SOURCE = (Path(__file__).resolve().parents[1] / "modules/nav2/robot_navigator.cpp").read_text()


def method(name, next_name):
    start = SOURCE.index("void RobotNavigator::" + name + "(")
    end = SOURCE.index(next_name, start) if next_name else len(SOURCE)
    return SOURCE[start:end]


class Nav2LifecycleContract(unittest.TestCase):
    def test_new_goal_resets_stale_result_before_dispatch(self):
        body = method("sendGoal", "template <typename T>\nvoid RobotNavigator::goalResponseCallback")
        reset = body.index("result_code_ = rclcpp_action::ResultCode::UNKNOWN;")
        dispatch = body.index("async_send_goal(")
        self.assertLess(reset, dispatch)
        self.assertIn("lock(result_mutex_)", body[:dispatch])
        self.assertIn("task_completed_ = false;", body[:dispatch])

    def test_cancel_request_never_publishes_terminal_result(self):
        body = method("cancelTask", "bool RobotNavigator::isTaskComplete")
        self.assertIn("cancel_active_goal_", body)
        self.assertNotIn("async_cancel_all_goals()", body)
        self.assertNotRegex(body, r"(?m)^\s*(?:result_code_|task_completed_)\s*=")

    def test_deferred_cancel_checks_generation_at_dispatch(self):
        body = method("sendGoal", "template <typename T>\nvoid RobotNavigator::goalResponseCallback")
        closure = body.split("cancel_active_goal_ = [this, generation, action_client, handle]", 1)[1]
        closure = closure.split("};", 1)[0]
        self.assertIn("lock(active_goal_mutex_)", closure)
        self.assertIn("generation != active_goal_generation_", closure)
        self.assertLess(closure.index("generation != active_goal_generation_"),
                        closure.index("async_cancel_goal(handle)"))

    def test_rejected_goal_is_completed_as_unknown(self):
        body = method("goalResponseCallback", "template <typename T>\nvoid RobotNavigator::feedbackCallback")
        rejected = body.split("if (!goal_handle)", 1)[1].split("} else {", 1)[0]
        self.assertIn("result_code_ = rclcpp_action::ResultCode::UNKNOWN;", rejected)
        self.assertIn("task_completed_ = true;", rejected)

    def test_stale_goal_callbacks_cannot_update_replacement(self):
        # Every asynchronous Nav2 callback must reject a superseded generation
        # before touching feedback, task completion or terminal result.
        for name, next_name in (
            ("goalResponseCallback", "template <typename T>\nvoid RobotNavigator::feedbackCallback"),
            ("feedbackCallback", "template <typename T>\nvoid RobotNavigator::resultCallback"),
            ("resultCallback", ""),
        ):
            with self.subTest(callback=name):
                body = method(name, next_name)
                guard = "if (generation != goal_generation_.load()) return;"
                self.assertIn(guard, body)
                self.assertLess(body.index(guard), body.index("std::lock_guard", body.index(guard)) if "std::lock_guard" in body else len(body))

    def test_terminal_result_only_from_result_callback(self):
        body = method("resultCallback", "")
        self.assertIn("result_code_ = result.code;", body)
        self.assertIn("task_completed_ = true;", body)


if __name__ == "__main__":
    unittest.main()
