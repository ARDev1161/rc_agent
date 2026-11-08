# RC API v2 — cross-repository implementation plan

> **Source of truth for all AI contributors.** Read this plan and `AGENTS.md` before making changes. Update status and handoff notes in the *same commit* as functional work. Do not claim a phase complete without passing its acceptance checks.
>
> Coordinated branches: [AmurCore](https://github.com/ARDev1161/AmurCore/tree/feat/rc-api-v2) · [rc_agent](https://github.com/ARDev1161/rc_agent/tree/feat/rc-api-v2) · [robot_orchestrator](https://github.com/ARDev1161/robot_orchestrator/tree/feat/rc-api-v2) · [ros2_rknn_llm](https://github.com/ARDev1161/ros2_rknn_llm/tree/feat/rc-api-v2). The canonical architectural decisions are in [AmurCore/PLAN.md](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/PLAN.md); each repository keeps a locally actionable copy of the plan.

## Product contract (approved requirements)
- **AmurCore is an operator workstation, NOT a robot controller, LLM planner, PDDL engine or fleet scheduler.** Operator can select one of many connected robots and use joystick/camera/map; give direct Nav2 goals; submit text/audio to an orchestrator; observe autonomous tasks.
- **rc_agent is a configurable protocol gateway** between the workstation and *pre-existing* robot ROS 2 topics/services/actions or local gRPC endpoints. Never put inverse kinematics, gait, balancing, motor drivers, path planning, task scheduling, or LLM processing in rc_agent.
- Control hierarchy: physical/motor layer outside rc_agent; direct 6-DOF velocity and Cartesian end-effector pose; supervisory navigation; semi-autonomous language tasks; robot-initiated autonomous tasks. Use supported-axis declarations; do NOT add per-joint operator control.
- Assignment: explicitly selected robot -> that robot; otherwise ask external fleet allocator/OpenRMF. AmurCore never decides priority/preemption/robot allocation internally.
- **Busy teleop interception:** display active task and exactly three choices: back (unchanged), pause (resume after teleop release), cancel (do not auto-resume). Never fake pause by silently canceling Nav2; implementing backend pause/resume is a distinct acceptance gate.
- Tasks are hierarchical at any depth. Show one-line summary + expandable child states; report LLM/PlanSys2/Nav2 stages with real task/parent IDs. “accepted by planner” is NOT “execution succeeded”.
- Asynchronous bidirectional conversation; AmurCore sends text or recorded audio unchanged via rc_agent; robot can initiate prompts days later; correlate conversation and task. Robot owns history and logs; AmurCore can view/export.
- Each robot has its own 2D OccupancyGrid/pose/map; keep current map/NavDialog, waypoints/patrol and GStreamer video. Optional fused 2D maps are provided by external systems, not in rc_agent; 3D deferred.
- Robot sends a configured **capability/endpoint profile** (only abstract capabilities to AmurCore; do not disclose ROS endpoint wiring unless needed for diagnostics). No dynamic UI framework: use existing widgets, simple optional action/parameter controls.
- **Traffic:** heartbeat periodic; on connect/reconnect send coherent snapshot; thereafter publish battery/pose/health/tasks only on meaningful changes (thresholds/debounce configurable), never compare whole maps on CPU. Video is OFF for non-selected robots and GStreamer sender should stop or be gated. Inactive robots remain connected; low-rate health if necessary. Do not poll unchanged data forever.
- VPN works; no cloud broker or public-internet infrastructure in scope for v2. No assumption that all ROS actions/services can be introspected without an adapter.
- C++17, Qt5 and ROS 2; no dependency on proprietary tooling. Existing production endpoints and serialized wire fields are a compatibility boundary: additive protocol first, cutover only with tests.

## Ground-truth code pointers
- AmurCore `network/protobuf/{robot,base,navigation,map,video}.proto`, `network/server.cpp`, `amurcore.cpp`, `logic/{movements,navdialog,mapwidget}.cpp`.
- rc_agent `main.cpp`, `modules/{basecontrol,nav2,map}/`, `network/client.cpp`, `robot.cfg.in`, `params/basecontrol.yaml`.
- LLM `src/ros2_rknn_llm_node.cpp` and `src/planner_client.cpp`; planner `robot_planner/src/goal_gateway.cpp`. The planner gateway currently reports submitted/accepted/rejected; it does NOT track full execution.
- Existing AmurCore `AGENTS.md` must be preserved and extended, not discarded.
- `robot_orchestrator` is a **superproject with Git submodules**; modifications in `ros2_rknn_llm` do NOT automatically update the superproject's pinned SHA. Update the submodule pointer in a controlled later commit and do not make a main/master deployment claim without it.

## Work phases and gates (do not skip)
| Phase | State | Work (owning repo) | Acceptance / evidence |
|---|---|---|---|
| P0 Baseline + coordination | DONE | feature branches; AGENTS.md + PLAN.md in all affected repos; inventory of current flow | document base refs, no default branch changes |
| P1 Additive API skeleton | IN PROGRESS | synced minimal proto across AmurCore + rc_agent; agent config -> RobotInfo; first receipt/inspection on workstation; tests | same proto byte-for-byte, legacy `Controls`/`Sensors` fields untouched, `protoc` generates; old clients continue; config defaults safe |
| P2 Robot-side endpoint bindings | TODO | configurable ROS2 cmd_vel (Twist/TwistStamped), odom/map/TF/Nav2 action names, optional local gRPC adapters with allowlists; config validation | two robot profiles, different endpoint names, no code edits; unknown types rejected; action cancellation status correct |
| P3 Multi-robot isolation | TODO | AmurCore per-connection sessions/state, selection/UI routing, per-robot maps+task+video, connection lifecycle | two concurrent agent simulators cannot cross-control or mix map/sensors; selection routes correctly; disconnect releases velocity authority |
| P4 Task + dialogue transport | TODO | correlate task/parent/stage ID, status snapshot/event, text/audio forwarding, asynchronous robot-originated chat + history/log requests | planner accept != completed; expandable stages, late prompt, disconnect/reconnect; bounded audio chunks |
| P5 Efficient streams + teleop preemption | TODO | snapshot + changed state, rate/hysteresis; independent map updates, video enable on selected robot; explicit pause/resume/cancel with backend acknowledgements | idle video network=0; no stale velocity after disconnect; no task resumes after cancel |
| P6 Cross-repo verification | TODO | integration tests (2 robots, Nav2 mock, LLM/PlanSys2 mock, GStreamer), docs, migration instructions and draft PRs | build, unit + contract tests green; observed status trace; compatibility matrix; no unreviewed merges |

## Required contract details before wiring P1/P2
- Identify `robot_id` stably from config; reject duplicate live IDs (no silent overwrite), session per authenticated stream, protocol major/minor; reconnect increments session generation.
- Velocity: linear XYZ, angular XYZ in explicit coordinate frame and **SI units** on v2 wire; existing joystick’s -255..255 legacy scale must be translated only at an explicit boundary; supported axes sent by robot. Command validity/deadman/watchdog mandatory before enabling v2 teleop. Keep local controller safety authoritative.
- Task: unique `task_id`, optional parent ID, source, typed lifecycle; accepted/queued/running/succeeded/failed/canceled/paused distinct. Reject duplicate command effects; no claim exactly-once.
- For ROS topic publish, `dispatched` means emitted, NOT physical completion. True success only from action/result/state feedback.
- Streaming: changes by semantic value with numeric threshold, event ID/sequence for replay; no expensive occupancy-grid comparison.
- Video: robot supplies ready-to-use GStreamer connection pipeline or URL; workstation chooses camera; default OFF when robot not selected.
- Security: allowlist configured endpoints; never accept arbitrary remote-supplied ROS topic, gRPC service/method, scripts, shell commands, or plugin paths; use local config authority.
- Pause/resume semantics and OpenRMF allocation are outside rc_agent; if upstream executor cannot pause, explicitly report unsupported and disable Pause button.

## Implementation rules for every contributor
1. Before editing: read **this file, `AGENTS.md`**, relevant source code and other-repo contract. Check outstanding `IN PROGRESS` items; take a small well-defined scope. Do not reimplement features already present.
2. One repository/phase per focused commit; no unrequested merge to `dev`/`master`. Keep existing ROS2 Nav2 and GStreamer functional. Never delete legacy messages, wire field numbers, or endpoints in P1.
3. Update this file's phase checklist and **Handoff** with exact changes, commit SHA or URL, tests actually executed, blockers, and next task. If change touches shared proto, sync exact definitions to BOTH repositories and prove equality.
4. Tests required: `protoc` for modified proto (when available); `cmake`/colcon on matching target OS; regression for legacy Nav2/map/video; deterministic unit tests. If dependencies absent, record `NOT RUN` with reason, never claim green.
5. Do not invent support for a ROS service/action type: register a tested adapter or fail closed. No unbounded binary audio in gRPC streams. Control must expire safely.
6. Before any change to the user-approved product contract, update an **Open decisions** entry and obtain explicit confirmation rather than silently introducing a framework.
7. GitFlow: `main/master` stable, `dev` integration where present, `feat/rc-api-v2` for this work, optional `fix/*` focused bugfixes. Conventional commits. Document owner/repo, base and deps.

## Open decisions and constraints
- Dedicated shared `rc_protocol` repository vs mirrored proto: defer; **mirror identically for P1** to avoid build-system/repo proliferation.
- Exact authentication/TLS implementation: not in P1, but do not expose public network interfaces insecurely.
- Task state replay/retention policy: define before P4; robot owns logs and canonical task state.
- ROS2 interface types and parameter mapping for nonstandard robots: P2 built-in adapters first; no dynamic script injection.
- OpenRMF integration later through external adapter, not rebuilt inside AmurCore.

## Handoff (append on each change)
- 2026-11-01: Cross-repository `feat/rc-api-v2` branches created from AmurCore:dev, rc_agent:dev, robot_orchestrator:master, ros2_rknn_llm:master. Product requirements captured above. P1 starting. No production branch changed.

### rc_agent current lane
- First: read `robot.cfg.in`, `params/basecontrol.yaml`, `main.cpp`. Some cmd_vel fields already configurable through ROS2 parameters; avoid recreating them. Make hardcoded odom/map/TF/Nav2 endpoints configurable in P2.
- P1 work: parse agent-owned capabilities config, validate stable robot id; announce via additive `Sensors.robot_info` during old stream until v2 session migration. Agent does not implement model-specific controls, IK, balance, actuators.
- Use ROS2 Humble-compatible C++17. Tests: `colcon build --packages-select rc_agent`; `colcon test --packages-select rc_agent`; use a ROS2 stub if hardware unavailable.


## P1 progress checkpoint — 2026-11-01

**Implemented in this coordinated branch**
- [x] Same `network/protobuf/rc_api_v2.proto` in AmurCore and rc_agent; additive `Robot.Sensors.robot_info = 9`; legacy Controls, Nav2, Map and Video field numbers untouched.
- [x] rc_agent: `robot.cfg.in` optional `Amur.API.{robot_id,display_name,capabilities,velocity_axes}`; list reader with type validation; metadata populated *before* starting the network stream; a blank robot ID defaults to legacy mode; ROS2 param `robot_config_path`.
- [x] AmurCore: existing Robot Info dialog shows advertised capability list and axes; receives under existing sensor mutex. **Still ONE shared session**, not fleet-safe; multi-robot work remains P3.
- [x] rc_agent: two new config parser GTests authored (valid list / invalid mixed list) but **NOT EXECUTED**.
- [x] ros2_rknn_llm: planner `kAccepted` now publishes JSON `status: accepted`, not `succeeded`; README clarified. Actual executor completion reporting remains a P4 deliverable.
- [x] robot_orchestrator: feature-branch gitlink updated to commit `aa7c4b28bd2bc0462cfe52daa9c795d257abbb24` of ros2_rknn_llm (do not confuse with submodule master).

**Validation completed:** repository source inspections; checked BOTH new proto texts byte-for-byte identical; BOTH robot.proto texts byte-for-byte identical; verified legacy wire tag positions, agent descriptor assignment, AmurCore reader, LLM status mapping, valid C++ test fixture string escaping, and superproject gitlink. All 10 checks PASSED (static inspection only).

**NOT RUN / blockers:** `protoc`, `grpc_cpp_plugin`, ROS2/`colcon`, Qt5, libconfig++ headers and full build/test dependencies are not installed in the available execution environment. Neither protobuf code generation nor C++ compilation, GTests, Nav2/GStreamer smoke tests, or multi-robot sessions have been executed; do not claim P1 accepted/green.

**Next work (in order):**
1. On a development machine with dependencies: `protoc -I network/protobuf --descriptor_set_out=/tmp/rc-v2.pb network/protobuf/rc_api_v2.proto network/protobuf/robot.proto` on both checkouts; `colcon build --packages-select rc_agent` + `colcon test --packages-select rc_agent` (inside sourced ROS 2 workspace); Qt5 Debug CMake build of AmurCore. Fix issues before advancing.
2. Functional P1 smoke: set unique robot ID in agent config; connect; open Robot Info; verify capability/axis labels; connect old agent against new AmurCore and new agent against old AmurCore; check Nav2 map/joystick/GStreamer regression.
3. Add focused RobotInfo validation and reconnect snapshot test; only then mark P1 DONE and start P2 with configurable hardcoded odom/map/TF/Nav2 action endpoints.
4. Security and safety before v2 velocity enablement: don't route raw v2 velocities to motors; add TTL, session binding, deadman and watchdog in P5.

**Known limitations:** adding `RobotInfo` to the old shared `Sensors` stream is a transitional bridge only; it is not safe for two simultaneous robots until P3 isolation. `Velocity`, task/chat and stream-selection messages are **schemas only**, not wired into runtime. Empty robot ID intentionally means no announcement, even if capabilities are configured. P1 remains IN PROGRESS pending builds and compatibility smoke tests.


## P1 CI and regression checkpoint — 2026-11-01
**New automatic tests** (on `feat/rc-api-v2` pushes and PRs):
- AmurCore + rc_agent `tests/test_proto_contract.py`: exact legacy `Controls`/`Sensors` field numbering, additive `robot_info` field 9, legacy Nav2/map/video services, protobuf roundtrips, TaskStatus acceptance semantics, Chat oneof and velocity TTL. C++ `tests/proto_cpp_smoke.cpp` compiles generated protobuf classes and verifies real serialization. GitHub Actions compares the mirrored `robot.proto` and `rc_api_v2.proto` byte-for-byte.
- rc_agent existing + new `ConfigStringList` GTests compiled/executed against libconfig++. CTest typo corrected to point to `rc_agent_unit_tests`.
- AmurCore `qt-app-build` compiles the complete Qt5 application using system packages; rc_agent `ros2-humble-build` compiles/tests the full ROS2 Humble package using `ros:humble-ros-base`. These are separate from quick contract jobs, not substitutes.
- ros2_rknn_llm `tests/test_planner_status_contract.py` asserts accepted != completed, cancellation/rejection reporting and output JSON fields. This is a source-contract check without RKLLM hardware, not an end-to-end LLM runtime test.
- robot_orchestrator workflow verifies the pinned `ros2_rknn_llm` gitlink tracks the feature branch and runs the pinned regression tests.
**Actual green runs (historical SHAs):** [AmurCore protocol #4](https://github.com/ARDev1161/AmurCore/actions/runs/41259526950), [rc_agent protocol + config #4](https://github.com/ARDev1161/rc_agent/actions/runs/41259529672), [ros2_rknn_llm status #1](https://github.com/ARDev1161/ros2_rknn_llm/actions/runs/41259502714), [robot_orchestrator gitlink #3](https://github.com/ARDev1161/robot_orchestrator/actions/runs/41260175233). Future commits require a fresh check of HEAD, these are not sufficient to mark P1 complete.
**Issues discovered by real CI, fixed on feature branches:** old Ubuntu protoc 3.12 needs proto3-optional flag; system Protobuf/gRPC need CMake module/library fallbacks; rc_agent grpcClient incorrectly tried to lock shared_ptr<mutex> instead of *mutex; downstream MapNode and BaseControl/RobotNavigator integration had inconsistent mutex ownership; VideoSourceRelay::convertAndScale was static but uses instance format settings; AmurCore NavDialog relied on `absl::string_view` even with older protobuf. These were hidden by protocol-only checks.
**Full-build verification:** still IN PROGRESS / NOT YET GREEN as of this checkpoint. Look at latest GitHub Actions run and fix all remaining compiler/CTest errors; do not assert production or hardware behavior without smoke testing. All changes stay on `feat/rc-api-v2`, P1 remains IN PROGRESS until full build, compatibility and manual Nav2/map/video checks.


## Verified P1 automated CI checkpoint — 2026-11-01 (after actual Actions execution)

**Automated checks that passed** (live GitHub Actions, not just local inspection):
- **AmurCore** full Qt5 application build and protobuf C++/Python wire-contract jobs: [run #10, both jobs success](https://github.com/ARDev1161/AmurCore/actions/runs/41260332212), feature commit `463a059b139d444c0da8607a060b213706d95e52`. Build included RobotInfoDialog + generated v2 API.
- **rc_agent** complete ROS 2 Humble `colcon build` and `colcon test`, config GTests, protobuf C++/Python compatibility: [run #16, all three jobs success](https://github.com/ARDev1161/rc_agent/actions/runs/41260131515), feature commit `bfa6bb9d1c317b789fc18acf077dde76b38b2c4e`.
- **ros2_rknn_llm** source contract for acceptance vs completion (not RKLLM hardware integration): [run #1, success](https://github.com/ARDev1161/ros2_rknn_llm/actions/runs/41259502714).
- **robot_orchestrator** pinned feature LLM submodule and source contract: [run #4, success](https://github.com/ARDev1161/robot_orchestrator/actions/runs/41260442446). This can require a new gitlink commit after updates to child documentation; verify HEAD pin matches child HEAD.

**Real bugs discovered and fixed via full build**: mutex shared-pointer dereferencing, MapNode constructor ownership, nonstatic video resize state, proto3 optional flag, CMake Protobuf/gRPC system package compatibility, Qt generated protobuf include path, Abseil enum string_view misuse, CTest typo. Regressions now caught by CI. All modified production source files included in the successful build commits above.

**P1 status remains IN PROGRESS (manual gates)**: automatic compilation/testing is green, but hardware/VPN/manual cross-version sessions remain unverified. Still required: (a) configure nonempty stable ID on real rc_agent and inspect Robot Info in running AmurCore, (b) old/new combinations and Nav2 motion/map/waypoints/GStreamer smoke, (c) safety and multi-session isolation phases P3/P5. Do not claim these pass until executed.

**What next AI must do**: use CI jobs as mandatory merge gates, investigate any red latest run, verify feature HEAD with checks before PR. Then run robot/GUI manual acceptance checks for P1 and record evidence; proceed to P2 only after gates. `dev`/`master` are unchanged.


## P1 loopback / compatibility expansion — 2026-11-01

New headless integration tests complement the earlier protobuf-only and full-build checks:
- AmurCore `tests/grpc_server_smoke.cpp`: launches the **actual** `grpcServer` on an ephemeral localhost port and exercises legacy unary DataExchange, v2 RobotInfo and velocity axes on the same sensor wire, bidirectional control exchange, callback, 2D map streaming and UI mutex availability during an idle stream. The implementation in `network/server.cpp` now reads and writes **outside** the GUI mutex, copying only sensor/control snapshots under the mutex. This prevents operator UI blockage on network reads.
- AmurCore `tests/test_legacy_wire_interop.py`: builds *unmodified* `dev` protobuf Python modules separately and runs a second interpreter to prove old→new and new→old wire parsing, including old readers ignoring optional `robot_info`. Automated CI [run #16](https://github.com/ARDev1161/AmurCore/actions/runs/41262624885) passed the cross-version contract and full application build.
- AmurCore `tests/robot_info_dialog_smoke.cpp`: offscreen Qt5 UI test checks descriptor display, second robot replacing the first, and fallback when no v2 descriptor. The `RC_API_V2_TESTING` CMake option keeps tests optional in normal builds; CI enables it.
- rc_agent `tests/grpc_client_smoke.cpp`: real `grpcClient` against a loopback mocked operator, validates unary and streaming handoffs, maintains ROS mutex availability during a stalled reply and verifies the initial snapshot on a newly created channel. The transport no longer requires `GRPC_CHANNEL_READY` before first send: gRPC's Write/Read handle CONNECTING and reconnection correctly. All four jobs in [rc_agent CI #22](https://github.com/ARDev1161/rc_agent/actions/runs/41262775490) **passed** (ROS2 Humble colcon build/test, config GTest, Python/C++ protobuf compatibility, real client loopback).
- Manual pass criteria and command examples: [AmurCore/docs/P1_MANUAL_ACCEPTANCE.md](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/docs/P1_MANUAL_ACCEPTANCE.md).

**State:** P1 remains IN PROGRESS pending real-robot/VPN, video, Nav2 and old↔new live application smoke. Hardware tests have NOT been run here. Do not mark P2 active or P1 done without manual evidence. P5 changed-only telemetry requires an explicit separation of control response latency from telemetry emission to avoid delaying joystick commands; do not blindly suppress identical sensor requests in the legacy lockstep RPC.
**Handoff:** inspect the most recent AmurCore `RC API v2 contracts` run (the one containing offscreen Qt test and strengthened mutex regression); if red, open the failed job log and fix. Then run manual checklist and record results. Do not merge `dev`/`master`.


## P1 current automated acceptance — verified on GitHub Actions, 2026-11-01

| Repository | Last tested production-code revision | Actual CI result |
|---|---|---|
| AmurCore | `65f47b8025ff1e5b3f9c8eb90b13587ff27a7f32` | [#20](https://github.com/ARDev1161/AmurCore/actions/runs/41262962581): **SUCCESS**, full Qt5 build + C++ gRPC server/idle-mutex + offscreen RobotInfo Qt + independent dev/v2 protobuf interop |
| rc_agent | `fae3ebddc810e76b1532d3db65512f7ac4807ef6` | [#25](https://github.com/ARDev1161/rc_agent/actions/runs/41263125743): **SUCCESS**, all four jobs: ROS2 Humble build/colcon tests, config GTests, C++ gRPC loopback client, Python/C++ protobuf contract |

**Production fixes** included in the successful commits: AmurCore releases the GUI mutex during network read/write; rc_agent copies sensor/control snapshots under its mutex but performs blocking RPC I/O outside it. All client streams can start when gRPC channel is CONNECTING; readiness is determined by actual Write/Read, preventing an empty initial snapshot. Invalid libconfig scalar types (robot ID/name and numeric/bool settings) return failure rather than an uncaught `SettingTypeException`. A dedicated GTest verifies no output mutation on invalid input.

**Tests**: legacy↔v2 unknown-field parsing, command/sensor/map streaming, callback, idle transport liveness, first message on fresh channel, UI descriptor replacement/fallback, malformed scalar/list config. Automated tests run in CI and all passed as indicated above; no test was run against the user's real robot, physical joystick, live Nav2/Map/TF2, VPN or GStreamer sender.

**P1 state: IN PROGRESS, hardware/manual acceptance outstanding.** Execute `docs/P1_MANUAL_ACCEPTANCE.md` (AmurCore repo) and capture logs and actual pass/fail before moving to P2. `dev`/`master` remain untouched. Do not misinterpret this automated gate as complete fleet, real telemetry change-only gating (P5), or live operator task control.


## P1 regression hardening — 2026-11-08
- P1 remains **IN PROGRESS**; P2 is not authorized until the manual acceptance checklist is completed.
- AmurCore test commit `1bcc98b2757a62d40b56f46665c4c7d0bdeec7b4`: offscreen RobotInfo regression now checks that a descriptor without stable robot_id cannot leak the previous robot's capabilities, name, or axes.
- rc_agent test commit `ce4394199df2d40a927508d49f4bd3562cff0010`: gRPC client loopback now checks that both unary and first stream snapshot deliver capability and axis along with robot ID; mock observation is mutex-protected to avoid a data race.
- Shared proto and legacy wire definitions **unchanged**. The above are test-only changes, not proof of real robot behavior.
- CI: check latest feature HEAD after this PLAN commit and record actual job outcomes; prior green runs are historical, not evidence for these new commits. Local Qt5/ROS2 builds and physical/VPN/Nav2/GStreamer tests: **NOT RUN** in this tool environment.
- Next: review feature HEAD CI jobs, fix any regression; then execute P1-01 through P1-08 with the physical robot or approved equivalent and record evidence. Do not mark P1 DONE without manual acceptance.


## P1 gRPC loopback regression fix — 2026-11-08
- Before fix, [rc_agent CI #42429638780](https://github.com/ARDev1161/rc_agent/actions/runs/42429638780) failed only `grpc-client-loopback` (20s timeout after `Client started`); protobuf, config and ROS2 Humble jobs succeeded. Root cause in `tests/grpc_client_smoke.cpp`: `OperatorMock::capture()` recursively called itself while holding `observed_mutex` instead of assigning `observed_robot_id`, causing deadlock.
- Replaced recursion with capture of the stable ID and reset of observed capability/axis before every snapshot; streaming uses the same mutex-protected capture. Added a legacy-after-v2 unary regression proving no old descriptor leaks across the same client session.
- Scope: P1 automated transport test only. Production client, protobuf wire and user-facing behavior unchanged. **CI pending for the fixing commit** at time of authorship; check actual `feat/rc-api-v2` HEAD run and fix any remaining red jobs.
- Do not infer physical robot/VPN/Nav2/GStreamer behavior from loopback; P1 stays **IN PROGRESS** pending P1-01..08 acceptance.


## P1 CI verified after descriptor regression — 2026-11-08
- Working source/test commit `da2b081dbe25a4bf8c560cfa3db7eb0f0ac18252`: [RC API v2 CI run #42431794847](https://github.com/ARDev1161/rc_agent/actions/runs/42431794847) **SUCCESS** at this exact test revision. All four jobs passed: ROS2 Humble colcon build/test, config GTests, protobuf compatibility and the fixed real gRPC client loopback (previous 20s hang eliminated).
- Paired peer tested source/test commit `bc6d90648d5bd0dfbedb984373524fc1c07857e1`; the common `robot.proto` and `rc_api_v2.proto` remained unchanged and mirrored.
- `documentation` revision `54797a67276f1b5e82e946c03c2846051962269d`: [Sphinx/Doxygen/Pages run #42431880321](https://github.com/ARDev1161/rc_agent/actions/runs/42431880321) **SUCCESS**, generated API source checkout pinned at tested commit during build. Documentation remains separate from production branches.
- This is **CI-only evidence**, not P1 hardware acceptance. P1 remains **IN PROGRESS** with P1-01..08 live robot/VPN/joystick/map/TF2/Nav2/GStreamer checks outstanding. No P2 implementation is authorized before manual gate; next safe action is to record real smoke results when robot is available, or add only genuinely useful deterministic P1 regressions. Do not misinterpret an untested `PLAN.md`-only successor commit as a source-code CI pass.
