# rc_agent

**robot gateway (ROS 2/C++17)** — reference for the RC API v2 migration.

rc_agent is a ROS 2/C++17 gateway on the robot. It reports agent-configured capabilities and bridges existing ROS 2 topics/actions and gRPC connections. It does not implement inverse kinematics, gait, motor control, LLM execution or fleet allocation.

**Migration status:** P1 additive descriptor transport is implemented and has historical green Qt/ROS2, Protobuf and loopback CI evidence. Hardware/VPN, live joystick, Nav2, map and GStreamer acceptance is still outstanding. Do **not** infer P2–P6 runtime support from protobuf declarations.

## Documentation map

- [Architecture](architecture.md) — boundaries and message flow
- [Getting started](getting_started.md) — current supported setup
- [RC API v2 protocol](protocol.md) — implemented wire and future schemas
- [Validation and compatibility](validation.md) — verified versus manual tests
- [C++ reference](api.rst) — Doxygen/Breathe, generated from working code
- [Standalone Doxygen browser](https://ardev1161.github.io/rc_agent/doxygen/index.html) — classes, files and source browsing
- [Docs build and publishing](deployment.md)

~~~{toctree}
:caption: Using rc_agent
:maxdepth: 2

getting_started
architecture
protocol
validation
~~~

~~~{toctree}
:caption: API and development
:maxdepth: 2

api
deployment
~~~

The website and its CI are maintained **only in the `documentation` branch**. The C++ reference is automatically generated from a separate checkout of [the working `feat/rc-api-v2` branch](https://github.com/ARDev1161/rc_agent/tree/feat/rc-api-v2). The displayed source revision in the website title comes from the checkout used for the build.
