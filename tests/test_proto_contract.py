"""Wire compatibility tests for the additive RC API v2 migration.

Requires generated Python protobufs in PYTHONPATH. See .github/workflows/rc-api-v2.yml.
"""
import unittest

import base_pb2
import map_pb2
import navigation_pb2
import rc_api_v2_pb2
import robot_pb2
import video_pb2


class LegacyWireContract(unittest.TestCase):
    def test_existing_controls_field_numbers_unchanged(self):
        actual = {f.name: f.number for f in robot_pb2.Controls.DESCRIPTOR.fields}
        self.assertEqual({
            "cameraServos": 1, "wheelMotors": 2, "handMotors": 3,
            "baseControl": 4, "navControl": 5, "light": 6,
            "system": 7, "video_config": 8,
        }, actual)

    def test_existing_sensors_fields_and_additive_robot_info(self):
        actual = {f.name: f.number for f in robot_pb2.Sensors.DESCRIPTOR.fields}
        self.assertEqual({
            "wheelEncoders": 1, "handEncoders": 2, "baseControlStatus": 3,
            "navControlStatus": 4, "batteryState": 5, "imu": 6,
            "CPUTemperature": 7, "video_stream": 8, "robot_info": 9,
        }, actual)
        self.assertEqual(
            robot_pb2.Sensors.DESCRIPTOR.fields_by_name["robot_info"].message_type.full_name,
            "rc.api.v2.RobotInfo",
        )

    def test_legacy_navigation_map_video_and_services_exist(self):
        self.assertEqual(base_pb2.BaseControl.DESCRIPTOR.fields_by_name["linearVelocity"].number, 2)
        self.assertEqual(base_pb2.BaseControl.DESCRIPTOR.fields_by_name["angularVelocity"].number, 3)
        self.assertEqual(len(navigation_pb2.NavCommandRequest.DESCRIPTOR.oneofs_by_name["command"].fields), 8)
        self.assertEqual(map_pb2.GetMapResponse.DESCRIPTOR.fields_by_name["map"].number, 1)
        self.assertEqual(video_pb2.VideoStream.DESCRIPTOR.fields_by_name["pipeline"].number, 1)
        self.assertIn("DataStreamExchange", robot_pb2.DESCRIPTOR.services_by_name["ClientOnRobot"].methods_by_name)
        self.assertIn("DataStreamExchange", robot_pb2.DESCRIPTOR.services_by_name["ServerOnRobot"].methods_by_name)


class V2Contract(unittest.TestCase):
    def test_robot_info_roundtrip_with_legacy_telemetry(self):
        sample = robot_pb2.Sensors()
        sample.robot_info.robot_id = "robot-001"
        sample.robot_info.display_name = "Dog"
        sample.robot_info.api_major = 2
        sample.robot_info.capabilities.add(id="manual.velocity", velocity_axes=["linear.x", "angular.z"])
        sample.batteryState.percentage = 0.85
        sample.navControlStatus.status = navigation_pb2.IN_PROGRESS
        restored = robot_pb2.Sensors.FromString(sample.SerializeToString())
        self.assertEqual(restored.robot_info.robot_id, "robot-001")
        self.assertEqual(list(restored.robot_info.capabilities[0].velocity_axes), ["linear.x", "angular.z"])
        self.assertAlmostEqual(restored.batteryState.percentage, 0.85, places=5)
        self.assertEqual(restored.navControlStatus.status, navigation_pb2.IN_PROGRESS)

    def test_acceptance_and_pause_are_distinct_from_completion(self):
        self.assertNotEqual(rc_api_v2_pb2.TaskStatus.ACCEPTED, rc_api_v2_pb2.TaskStatus.SUCCEEDED)
        self.assertNotEqual(rc_api_v2_pb2.TaskStatus.PAUSED, rc_api_v2_pb2.TaskStatus.CANCELED)

    def test_chat_oneof_clears_other_content(self):
        event = rc_api_v2_pb2.ChatMessage(text="Open door")
        self.assertEqual(event.WhichOneof("content"), "text")
        event.audio = b"\\x00\\x01"
        self.assertEqual(event.WhichOneof("content"), "audio")
        self.assertFalse(event.HasField("text"))

    def test_velocity_roundtrip_and_ttl(self):
        command = rc_api_v2_pb2.Velocity(valid_for_ms=300, sequence=9, frame_id="base_link")
        command.linear.x = 0.25
        command.angular.z = -0.4
        self.assertEqual(rc_api_v2_pb2.Velocity.FromString(command.SerializeToString()), command)
        self.assertEqual(command.linear.y, 0)


if __name__ == "__main__":
    unittest.main()
