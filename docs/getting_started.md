# Getting started

Use the [`feat/rc-api-v2` source branch](https://github.com/ARDev1161/rc_agent/tree/feat/rc-api-v2). Documentation itself is maintained separately and is **not** a buildable source branch.

## Build gateway

The gateway uses ROS 2 Humble and C++17. Put the repo into a sourced ROS 2 workspace and use `colcon build --packages-select rc_agent` and `colcon test --packages-select rc_agent`. Follow the [full ROS2 CI job](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/.github/workflows/rc-api-v2.yml) for build dependencies and flags.


## Enable v2 capability advertisement

The agent configuration is **libconfig**, not YAML. Add a stable unique `robot_id` to `robot.cfg` (the template is `robot.cfg.in`):

```text
Amur: {
  API: {
    robot_id = "test-robot-001";
    display_name = "Test robot";
    capabilities = ("manual.velocity", "navigation.nav2", "map.2d", "video");
    velocity_axes = ("linear.x", "angular.z");
  };
};
```

An empty `robot_id` intentionally disables v2 advertisement for legacy compatibility. These are **declared** capabilities, not proof that all proposed v2 command paths are active.

Start with an absolute config path inside a sourced ROS 2 Humble workspace:

```sh
ros2 run rc_agent rc_agent --ros-args -p robot_config_path:=/absolute/path/robot.cfg
```


## P2 trusted endpoint profiles (in progress)

Robot-specific ROS2 topic and Nav2 action names are configured in **`robot.cfg` (libconfig)**, not in remote protobuf messages. The following optional values preserve legacy defaults if omitted:

```text
Amur: {
  API: {
    robot_id = "robot-alpha";
    endpoints: {
      odom_topic = "/alpha/odom";
      cmd_vel_topic = "/alpha/cmd_vel";
      cmd_vel_type = "twist"; // or "twist_stamped"
      cmd_vel_frame = "base_link";
      cmd_vel_smoothed_topic = "/alpha/cmd_vel_smoothed";
      cmd_vel_nav_topic = "/alpha/cmd_vel_nav";
      map_topic = "/alpha/map";
      navigate_to_pose_action = "/alpha/navigate_to_pose";
    };
  };
};
```

`cmd_vel_smoothed_topic` and `cmd_vel_nav_topic` bind the **existing legacy auxiliary selectors** to this robot's ROS2 namespace; they do not allow a remote caller to supply arbitrary ROS2 topic names. Badly typed or empty configured endpoint values fail closed. ROS2 node parameters (for example speed scaling) remain separate from the libconfig endpoint profile.

Different robots should use distinct configured topic and action names. P2 tests verify independent profiles and real ROS2 message delivery, but do **not** establish P3 gRPC multi-robot session isolation or authorize v2 teleoperation. V2 velocity stays disabled until TTL/session/lease/deadman/watchdog are enforced.

## Compatibility first

P1 only adds `Robot.Sensors.robot_info = 9`. Existing `Robot.Controls`, map/Nav2 and video paths must continue to work. V2 `Velocity`, task/chat and stream selection are currently **schemas**, not production control paths. Do not enable raw v2 velocity forwarding without TTL, session lease, deadman and watchdog.
