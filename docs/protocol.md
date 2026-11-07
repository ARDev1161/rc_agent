# RC API v2 protocol

The canonical mirrored definitions are [`rc_api_v2.proto`](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/network/protobuf/rc_api_v2.proto) and [`robot.proto`](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/network/protobuf/robot.proto). The same files appear in [AmurCore](https://github.com/ARDev1161/AmurCore/tree/feat/rc-api-v2/network/protobuf). CI compares them byte-for-byte.

## P1 implemented: RobotInfo

`Robot.Sensors.robot_info = 9` adds a descriptor without changing legacy `Controls`, map, Nav2 or video field numbers. A nonempty agent-configured robot ID enables advertised API 2.0, display name, capabilities and velocity axes.

Older protobuf receivers ignore unknown field 9. Empty IDs retain legacy behavior.

## Defined, not yet wired into runtime

| Message | Intended use | Status |
| --- | --- | --- |
| `Velocity` | SI units, 6-axis, frame ID, sequence, TTL | Schema only; no v2 teleop publisher |
| `TaskStatus` | Unique ID, optional parent, queued/accepted/running/paused/succeeded/failed/canceled | Schema only |
| `ChatMessage` | Correlated text/audio (bounded chunks required) | Schema only |
| `CustomAction` | Typed parameters to locally allowlisted adapter | Schema only |
| `StreamSelection` | Selected camera/video and active/inactive transport | Schema only |

`ACCEPTED` is **not** `SUCCEEDED`. Remote agents must not receive arbitrary ROS topic, service, action, process or plugin paths. Future teleop requires short lived session authority/lease, TTL, deadman and watchdog before publishing.
