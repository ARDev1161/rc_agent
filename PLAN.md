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


## P1 real-peer CI gate (2026-11-08)
- New real-peer-grpc-loopback job compiles BOTH real AmurCore server and rc_agent client together using mirrored proto, runs the shared AmurCore scenario on ephemeral localhost with legacy/v2/legacy/reconnect snapshots, and fails on lost controls/descriptor or callback mutex blocking.
- Both repositories have the same test job; this branch checks out the other peer at feat/rc-api-v2. No production endpoints, proto, ROS2 bindings or motor control changed. CI is PENDING until both jobs finish; hardware acceptance P1-01..08 remains NOT RUN, P1 IN PROGRESS.


## Verified P1 cross-repository real-peer CI — 2026-11-08
- Tested source+CI revision `41db27715aa2e453742fb712b548b7bcdfe51dd2`: [RC API v2 workflow #42432945859](https://github.com/ARDev1161/rc_agent/actions/runs/42432945859) **SUCCESS** on that exact SHA. All 5 jobs pass: complete ROS2 Humble colcon build/test, protobuf, config GTests, existing client/mock loopback, and real rc_agent client ↔ AmurCore server integration.
- Paired real peer source/CI revision `8da7cc1303fb7e6687e7f889792bc260d771a309`. New integration job verifies legacy controls, v2 robot capabilities/axes over the unchanged RPC, legacy-after-v2 clearing, fresh-channel reconnect and nonblocking sensor callbacks. Initial CI attempts exposed build-time header-name collisions and were corrected explicitly; no production code or proto changed.
- Updated documentation in the separate documentation branch: [Pages run #42432873508](https://github.com/ARDev1161/rc_agent/actions/runs/42432873508) SUCCESS. Documentation will refresh its displayed source revision to this tested source SHA.
- **Acceptance**: P1 remains IN PROGRESS. These are headless in-process loopback tests, not hardware/VPN/real joystick/Nav2/map/GStreamer acceptance. Execute P1-01 through P1-08 and capture actual results before starting P2. No merges to dev/master.


## P1 gRPC retry/liveness hardening — 2026-11-08
- The unary transport previously retried even with tryConnectIfFailed=false, slept while holding the ROS sensor/control mutex, and had no per-attempt deadline. Updated network/client.cpp to snapshot telemetry under mutex, bound each unary attempt to 3s, return gRPC failures immediately when retries disabled, and only update controls on success; network I/O/backoff now runs outside mutex.
- Four legacy streaming RPC methods now respect the constructor retry policy after a failed stream; stoppedStream is atomic for cross-thread access. NetworkController forwards the provided tryConnectIfFailed argument instead of ignoring it.
- P1 only: no protobuf or actuator path changes. This does not implement stream cancellation while blocked in gRPC Read (still a separate lifecycle concern), nor P5 safe velocity lease/deadman. CI PENDING; no hardware/VPN/Nav2/map/video acceptance. P1 remains IN PROGRESS.

- Additional `tests/grpc_client_smoke.cpp` assertions: simulated first unary UNAVAILABLE with retry disabled returns once and preserves prior controls; retry-enabled unary recovers after one simulated transient failure with callback mutex available during backoff; unary and streaming one-shot policy both fail promptly. These cases run under the existing grpc-client-loopback GitHub Actions job; CI outcome pending.


## Verified P1 transport disconnect and retry regression — 2026-11-08
- Tested source SHA `29cda7bf4a390bb5f672feacdbcd4f70d880588a` — [GitHub Actions #42435001800](https://github.com/ARDev1161/rc_agent/actions/runs/42435001800) **SUCCESS** at exactly this revision. All 5 jobs: full ROS2 Humble colcon build/test, config GTests, protobuf, gRPC mock failure/recovery, real AmurCore/rc_agent integration.
- Tested peer `956f2b970885c65664067f8a5a84eea2dc080a91`; both projects compile real peer implementations over unchanged legacy gRPC and mirrored Protobuf.
- Docs source changes only in separate `documentation` branch, SHA `5690aff05a2d4fc02c89b8a75cfca37941aa0e04`, [Pages CI #42435184232](https://github.com/ARDev1161/rc_agent/actions/runs/42435184232) **pending at handoff**; check deployment and fix if red. The documentation tree must remain isolated and must not be merged to source branches.
- P1 still **IN PROGRESS**. Headless CI does not validate actual operator GUI workflow, physical joystick, hardware/VPN, Nav2/TF/map/video, safety watchdog or end-to-end lifecycle. Manual P1-01..08 remain **NOT RUN**; no P2 or default branch changes. Stream `stopStream()` still cannot cancel a blocking gRPC Read; atomic flag prevents retry only after the read unblocks. Do not claim fully resilient teleop until the P5 safety gates are implemented.


### Verified docs and code CI handoff — 2026-11-08
- Source/test SHA `29cda7bf4a390bb5f672feacdbcd4f70d880588a`: [RC API v2 CI #42435001800](https://github.com/ARDev1161/rc_agent/actions/runs/42435001800) **SUCCESS** (all 5 jobs).
- Docs-only SHA `5690aff05a2d4fc02c89b8a75cfca37941aa0e04`: [GitHub Pages CI #42435184232](https://github.com/ARDev1161/rc_agent/actions/runs/42435184232) **SUCCESS** (Sphinx/Doxygen isolation + Pages deploy).
- These success claims are scoped to exact source and docs revisions; plan-only handoff commit does not rerun source CI because workflows filter paths. P1 manual hardware/GUI/VPN/Nav2/GStreamer gates remain **NOT RUN**. Next permitted work: additional targeted P1 reliability tests or manual P1 acceptance; do not implement P2 until approved gate passes. Do not merge documentation or source feature branches.


### P1 failed-stream controls regression — 2026-11-08
- Added `tests/grpc_client_smoke.cpp` assertion after mocked UNAVAILABLE streaming RPC: previously valid controls remain unchanged, matching unary failure behavior. Source test commit [`ef600cfa`](https://github.com/ARDev1161/rc_agent/commit/ef600cfaa17e8ac9a34de13795d148ccc2481385). No proto or production runtime changes.
- CI for this test commit: NOT YET VERIFIED at authoring; check latest feature-head Actions (protobuf, config, client loopback, ROS2 Humble, real peer). Physical P1-01..08 remain NOT RUN; P1 IN PROGRESS, P2 blocked by manual acceptance.


## P1 stopStream blocked gRPC Read cancellation — 2026-11-08
- Stop previously set an atomic flag but could not interrupt a synchronous gRPC Read/Write. The gateway now tracks active grpc::ClientContext objects under a mutex for DataStreamExchange/MapStream/PoseStream/ZoneStream; context registration is rejected after stop, RAII unregistration occurs before destruction, and stopStream atomically sets stopped state and calls TryCancel for all active contexts.
- Reconnect requires a fresh grpcClient after stop; removed the per-stream automatic reset of the stopped flag. Backoff sleep polls the stop flag every 10ms. Transport shutdown does NOT implement v2 velocity watchdog/lease or motor safety. Protobuf and ROS2 endpoints unchanged.
- Mock-loopback test intentionally blocks Read and checks CANCELLED return, no retry/restart, and preserved last controls. CI **NOT YET VERIFIED** for this commit; inspect exact GitHub Actions feature HEAD. P1 IN PROGRESS; P1-01..08 hardware/VPN/GUI/Nav2/map/video NOT RUN.


### Verified P1 active-stream cancellation on current code — 2026-11-08
- Tested source/test SHA `598c6da8efd42bd0de120c7947f2c2a776677019`: [GitHub Actions run 42441827514](https://github.com/ARDev1161/rc_agent/actions/runs/42441827514) **SUCCESS** for all 5 jobs (including real `AmurCore` ↔ `rc_agent` gRPC and ROS2 Humble/colcon, config, mocked blocked-Read cancellation and protobuf tests). The cross-repo peer checked out was at `fd68254c572c20961d80be6ae089e5457feb38b2`.
- Docs-only `documentation` SHA `ebeb20835e3a25ee1a0bad2f02031db94234bc82`: [Sphinx/Doxygen/Pages run 42441942205](https://github.com/ARDev1161/rc_agent/actions/runs/42441942205) **SUCCESS**; no documentation sources were copied to working C++ branches.
- `stopStream()` now cancels blocking gRPC stream I/O; this is lifecycle cancellation, NOT proof of robot physical stop or v2 velocity lease/deadman/watchdog. A stopped client is terminal and must be replaced to reconnect. Current branch HEAD may differ by this PLAN-only handoff update; do not misrepresent the tested SHA.
- Phase **P1 IN PROGRESS**. Hardware/VPN/operator GUI/joystick/Nav2/map/video/old↔new live tests P1-01..08 remain **NOT RUN**. No P2 development or merge to dev/master permitted until gates; next meaningful task is physical/simulator manual acceptance when available, or a targeted P1 blocker regression.


### P1 concurrent streaming cancellation regression — 2026-11-08
- rc_agent test revision [5bb547e05e24df03610da0938964294278e41b6b](https://github.com/ARDev1161/rc_agent/commit/5bb547e05e24df03610da0938964294278e41b6b) adds two simultaneous blocked DataStreamExchange calls on the same real grpcClient, asserts stopStream() cancels both contexts, repeated stop is idempotent, CANCELLED statuses are returned, and previously valid controls remain unchanged. No runtime or shared proto changes; AmurCore real-peer source unchanged.
- CI: PENDING verification for the new test SHA; earlier successful runs are not evidence for this test. Hardware/VPN/joystick/Nav2/map/GStreamer and live dev↔v2 manual P1-01..08 remain NOT RUN. P1 IN PROGRESS; P2 not authorized. Next: inspect current rc_agent Actions job grpc-client-loopback, repair any failure, and update docs source SHA if needed.


## P1 coherent MapStream snapshot + transport error handling — 2026-11-08
- Audited real legacy Map/Pose/Zone streaming in `rc_agent/network/client.cpp`: MapStream previously read `map_seq` under `muMap`, unlocked, then locked again and called potentially blocking gRPC `stream.Write(*map_)` under the same mutex. This could block ROS2 map callbacks and send a payload inconsistent with the recorded sequence.
- Now copies `GetMapResponse` and its sequence atomically under `muMap`, then writes **outside** the lock; no full-map comparison or extra full-map scans. Map stream construction also releases the mutex before network calls. All three map/pose/zone streams now check `Write` and `Read` and exit the pump on failure so the existing retry policy can take over. Existing wire and ROS2 topics unchanged.
- CI: **NOT VERIFIED yet** for this commit. Paired AmurCore real-server smoke will verify initial map snapshot, updated sequence and cancellation over real localhost gRPC. P1 IN PROGRESS: P1-01..08 live GUI/joystick/TF/Nav2/map/GStreamer/VPN tests NOT RUN. P2 not started.


## P1 verified coherent map transport integration — 2026-11-08
- Source/test SHA `403d499735f5b996ee86011c0ee459569d05de11` [CI #42444234818](https://github.com/ARDev1161/rc_agent/actions/runs/42444234818) **SUCCESS** for all 5 jobs at that exact tested SHA; includes real `AmurCore` ↔ `rc_agent` gRPC test exchanging independent map instances with initial sequence 71 + pose and updated sequence 72 + pose, then canceling the map stream. ROS2 Humble colcon build/test, config GTests, mock gRPC loopback and mirrored protobuf checks also passed.
- Complementary peer source/test SHA: `0526a1e1c48a53d8a41b76e1810fabf223ad66a1`. No shared protobuf modification; `robot.proto` and `rc_api_v2.proto` remain mirrored. Existing legacy Nav2/video controls are preserved, not validated on real hardware.
- Documentation in separate `documentation` branch at `97e2bcf0ebe182e2f7c4552fe900779d02b44de2` [workflow #42444358908](https://github.com/ARDev1161/rc_agent/actions/runs/42444358908): Sphinx/Doxygen isolation **PASS**; **Pages deployment QUEUED at time of handoff**, do not claim complete deployment until it finishes. CI/Pages must be rechecked after GitHub completes pending deployment.
- **P1 IN PROGRESS**. Manual P1-01 through P1-08 on actual robot/VPN/GUI/joystick/Nav2/map display/GStreamer/old↔new binaries remain NOT RUN. P2 TODO, no dev/master merge. Next meaningful P1 action: review any pending red CI/deploy, then conduct manual acceptance or add targeted simulation only if it closes a specific uncovered failure.


### Completed documentation deployment for coherent map CI — 2026-11-08
- [Documentation branch revision `97e2bcf0ebe182e2f7c4552fe900779d02b44de2`](https://github.com/ARDev1161/rc_agent/commit/97e2bcf0ebe182e2f7c4552fe900779d02b44de2): [Sphinx/Doxygen + Pages run 42444358908](https://github.com/ARDev1161/rc_agent/actions/runs/42444358908) **SUCCESS** including deployment, updated from tested source `403d499735f5b996ee86011c0ee459569d05de11`. This supersedes the earlier *QUEUED* handoff in this file.
- Working-code [CI run 42444234818](https://github.com/ARDev1161/rc_agent/actions/runs/42444234818) **SUCCESS** on `403d499735f5b996ee86011c0ee459569d05de11`. No changes in this handoff to production code or mirrored protobuf. P1 remains IN PROGRESS until P1-01..08 live acceptance; P2 remains TODO.


## P1 consistent legacy PoseStream and ZoneStream transport — 2026-11-08
- Fixed rc_agent Pose/Zone stream factories holding `muMap` while creating the RPC. The map pointer is checked under the mutex, then the RPC is created outside it, matching the safe existing `MapStream` behavior.
- Fixed `ZoneStream` reading `zone_seq` and zone payload under **different** mutex acquisitions. The sequence and zone state are now captured in one map lock; network `Write`/`Read` remain unlocked, no full zone/map scan or new protocol field.
- Pair with AmurCore real-peer tests for initial and changed pose/zone data, independently of `MapStream`, and for cancellation of each. **CI PENDING for this source revision.** P1 manual physical/VPN/GUI/joystick/Nav2/GStreamer acceptance remains NOT RUN; P1 IN PROGRESS and P2 TODO.


## P1 verified separate PoseStream and ZoneStream loopback — 2026-11-08
- Tested source/test commit `f2ad035d2139a6a0b534ef2b1ea07f2aec5c5c6e`: [GitHub Actions #42445880201](https://github.com/ARDev1161/rc_agent/actions/runs/42445880201) **SUCCESS** for all 5 jobs, including true cross-repository `AmurCore` ↔ `rc_agent` gRPC, mirrored proto and complete ROS2 Humble colcon build/test + config GTests + existing mocked gRPC loopback.
- Peer checked out from `feat/rc-api-v2` during cross-repo CI: `7c8e1990aeacae6a23b4f7d8328e1d5608bc884a`. Under independent map storage, the real peer scenario proved MapStream initial and changed snapshots, then separate PoseStream initial/changed pose and ZoneStream initial/changed zone (map stream stopped), plus CANCELLED stream shutdown and preservation of map/pose fields. No changes to mirrored protobuf or legacy RPC IDs.
- Documentation-only branch tested SHA `caaa7326bed9375301843eb2b02b347d4c1fa049`: [GitHub Pages CI #42445977610](https://github.com/ARDev1161/rc_agent/actions/runs/42445977610) **SUCCESS** (strict Sphinx/Doxygen isolation and Pages publish). This PLAN-only successor commit does not need to change production/test source and must not be misrepresented as the tested code SHA.
- **P1 still IN PROGRESS**. These tests use headless localhost, not the user's live robot, VPN, Qt navigation workflow, physical joystick, Nav2/TF2/OccupancyGrid visualization, live old↔new binaries or GStreamer; manual P1-01..P1-08 remain **NOT RUN**. P2 still TODO, dev/master not touched.
- Next meaningful action: inspect pending feature-head CI on any future code changes and run P1 manual acceptance on an available simulation/robot; only extend automated P1 coverage to close an identified compatibility or deadlock failure. No artificial claims of complete v2 migration.


## P1 unary cancellation regression — 2026-11-08
- Source fix [981a7fbe](https://github.com/ARDev1161/rc_agent/commit/981a7fbe14b9f18d38ef4dbba28477f3db1abe89): DataExchange now registers its ClientContext with the existing stopStream cancellation registry, checks stopped state before retry and before applying controls, and polls stop during retry backoff. The existing 3-second per-attempt deadline remains. No shared protobuf or ROS2 mapping changes.
- Test [b05bab6d](https://github.com/ARDev1161/rc_agent/commit/b05bab6d1340d3de4ed9bd1bd63ef460f327b27b): mock operator deliberately blocks unary reply; stopStream must unblock it with CANCELLED, reject later unary attempts, and preserve previous controls. Paired AmurCore real-peer test at [60a6dd9e](https://github.com/ARDev1161/AmurCore/commit/60a6dd9e7f3261deb5a73ef1a539af7fac0a2f64) checks stopped unary behavior with the real peer.
- GitHub Actions [#42450762717](https://github.com/ARDev1161/rc_agent/actions/runs/42450762717) was IN PROGRESS at this handoff; do not claim success before inspecting final jobs. Local ROS2/Qt builds NOT RUN in this environment. P1-01..08 physical/VPN/GUI/Nav2/map/video/legacy live acceptance NOT RUN; P1 remains IN PROGRESS and P2 TODO.
- Next: inspect latest feature-head CI and fix any red job; verify AmurCore paired CI and documentation source revision in separate documentation branch. No dev/master merges.


### P1 unary cancellation CI verified — 2026-11-08
- Feature source/test revision `b05bab6d1340d3de4ed9bd1bd63ef460f327b27b` passed [Actions #42450762717](https://github.com/ARDev1161/rc_agent/actions/runs/42450762717): 5/5 jobs SUCCESS (ROS2 Humble colcon build/test, config GTests, protobuf, mock grpc-client-loopback, real AmurCore↔rc_agent gRPC). This supersedes the earlier PENDING handoff; branch also has a PLAN-only successor.
- Paired AmurCore [#42450768529](https://github.com/ARDev1161/AmurCore/actions/runs/42450768529): 3/3 jobs SUCCESS. Transport cancellation is not a robot stop/watchdog proof.
- P1 IN PROGRESS, P1-01..P1-08 manual gates NOT RUN; P2 TODO. No source/proto/default branch changes in this CI verification handoff. Next: targeted simulator/live acceptance; documentation remains isolated in `documentation`.


### P1 mixed RPC shutdown regression — 2026-11-08
- Production change [453be21f](https://github.com/ARDev1161/rc_agent/commit/453be21f643797a8450399c806898a9f7454c37f) adds a shutdown check while holding the controls mutex before applying a streaming reply; avoids reviving controls after stopStream. No proto or ROS endpoint changes.
- Test [4cb1faa9](https://github.com/ARDev1161/rc_agent/commit/4cb1faa9e0cc77e8ebbece37e44357413683bf40) exercises simultaneous blocked unary DataExchange and streaming DataStreamExchange on ONE grpcClient; stopStream must cancel both, prevent restart and preserve the previous control. Existing one-shot and concurrent-stream regressions remain.
- GitHub accepted both commits; new feature-head Actions result **NOT VERIFIED** (workflow run listing not available through connector; local git checkout fails DNS). Local build/test **NOT RUN**. Do not count earlier successful Actions as evidence for these commits. P1 IN PROGRESS, P1-01..08 live acceptance NOT RUN, P2 TODO. Next: obtain and inspect CI for 4cb1faa9, fix failures; update documentation source SHA only after CI verification.


### P1 stream retry shutdown regression — 2026-11-08
- [1a0f0a50](https://github.com/ARDev1161/rc_agent/commit/1a0f0a50930db89a5e0117bfedb74a717d8df129): runStreamWithRetry now returns CANCELLED if stopStream interrupts retry backoff after transient UNAVAILABLE, instead of leaking the transient failure as terminal shutdown status. [fc28ea70](https://github.com/ARDev1161/rc_agent/commit/fc28ea7083c9af2efb7d3c50e954470e999c9b6c): mock-server regression covers failed stream, shutdown during backoff and control snapshot preservation. No proto/ROS2/legacy endpoint changes.
- GitHub Actions for new source/test HEAD NOT VERIFIED at handoff; local build NOT RUN. Prior successful Actions #42450762717 and #42450768529 are historical SHAs, not evidence for this patch. P1 IN PROGRESS, manual P1-01..08 NOT RUN, P2 TODO. Next: inspect new CI and fix any failures; simulator/manual acceptance remains outstanding.


### P1 optional BatteryState telemetry (2026-11-08)
- Existing legacy `Sensors.batteryState = 5` and `Power.BatteryState` reused unchanged. [`9dbdd1f`](https://github.com/ARDev1161/rc_agent/commit/9dbdd1f1810edb0248a6c3de6f819d6a8f01a46c) / [`5ab8cf1`](https://github.com/ARDev1161/rc_agent/commit/5ab8cf13b1276831759b78c29f39ac3f2d2c266c): BatteryStateNode now writes converted ROS2 `sensor_msgs/msg/BatteryState` into shared Sensors under the existing grpcClient mutex instead of printing it. [`49e7f85`](https://github.com/ARDev1161/rc_agent/commit/49e7f858f0cbdb6db6a634245f58b74ec531320c) / [`a635a9f`](https://github.com/ARDev1161/rc_agent/commit/a635a9f560383e17221fea7fc0dd226595bcfb44): optional `Amur.API.battery_topic` libconfig string enables subscription; unset means no battery field, preserving robots without battery. [`c157a22`](https://github.com/ARDev1161/rc_agent/commit/c157a224dd507e1947048935cfe8b4626a6b806a) tests absent and populated battery on real mock unary gRPC transport. No proto edits, legacy field numbers unchanged.
- NOT VERIFIED: new feature-head ROS2 Humble compile/colcon and loopback Actions; local ROS2 build NOT RUN. UI rendering, freshness/NaN display and live ROS topic acceptance NOT RUN. P1 IN PROGRESS, P1-01..08 manual gates NOT RUN; P2 TODO. Next: inspect CI, correct failures, then integrate optional battery display without inventing zero charge.


### P1 optional battery display (2026-11-08)
- AmurCore source [`402d7e50`](https://github.com/ARDev1161/AmurCore/commit/402d7e50c2105760f866dd9b0f5dbfe3e5e71c3a) / [`4e893071`](https://github.com/ARDev1161/AmurCore/commit/4e893071730985d03a2e11aeb930614c4ebe183c): optional battery indicator in existing status bar; read `Sensors.batteryState` under gRPC mutex, hide if field absent or `present=false`, render finite 0..1 percentage and charging state, otherwise 'Battery: unknown'. UI updated on existing throttled sensor callback, no protobuf changes. Peer rc_agent has opt-in `Amur.API.battery_topic` ROS2 subscriber. Caution: ROS publisher loss/staleness timeout and disconnect-specific freshness not yet implemented; do not claim those are solved. CI/Qt build NOT VERIFIED, manual P1-01..08 NOT RUN; P1 IN PROGRESS, P2 TODO. Next: run current SHA Qt/ROS2 CI, fix compile/test failures, implement staleness handling and UI tests.


### P1 BatteryState freshness (2026-11-08)
- rc_agent [`ddac0d40`](https://github.com/ARDev1161/rc_agent/commit/ddac0d40a7fa27cb8216d5b6f4d38551d2d1d88a) and [`1ff18cac`](https://github.com/ARDev1161/rc_agent/commit/1ff18cacbe21ae54927f2203458b27cc177e8a05) add a steady-clock 5-second BatteryState receive timeout. ROS subscription and 250-ms timer use the default mutually exclusive callback group; on timeout the agent clears the optional `Sensors.batteryState` under the same mutex used for gRPC snapshots. Fresh data repopulates it. AmurCore already hides absent battery on its sensor-update event. No protobuf changes. WARNING: current CI/build and live topic timeout test NOT VERIFIED; UI disconnect behavior independent of stale ROS battery remains to be verified. P1 IN PROGRESS, manual P1-01..08 NOT RUN, P2 TODO.


### P1 disconnect battery guard (2026-11-08)
- AmurCore [`d47fa494`](https://github.com/ARDev1161/AmurCore/commit/d47fa49489f9280265be7dbe2b142dba510c651b) hides cached battery when robot list becomes empty; [`3db5ef21`](https://github.com/ARDev1161/AmurCore/commit/3db5ef21d22f11b3d93e4377f2b04a8222315ebc) additionally prevents stale sensor-refresh callback from re-showing it while no robots are known. This does not implement P3 per-session selection. rc_agent [`b372cb81`](https://github.com/ARDev1161/rc_agent/commit/b372cb817e8c90037ff0cc7ca5987f9c931cd606) documents battery freshness behavior in public Doxygen comments. No shared proto change. Current-head CI and Qt/ROS2 compile NOT VERIFIED in this handoff; hardware P1-01..08 NOT RUN. P1 IN PROGRESS; next: inspect current-head Actions, compile/test, add deterministic stale timeout regression, then manually check disconnect/reconnect.


### P1 ROS battery freshness regression (2026-11-08)
- rc_agent source commits [`bbf6202a`](https://github.com/ARDev1161/rc_agent/commit/bbf6202a3eb75f6bd5207296df48eecd5e50b54d), [`183908b1`](https://github.com/ARDev1161/rc_agent/commit/183908b121caefbcc40873fb910fd33577615024), [`a7c44e65`](https://github.com/ARDev1161/rc_agent/commit/a7c44e6546f1534c671f88dd4b15c55aef73ebb4): new actual ROS2 publisher/subscriber regression publishes 42%, waits for timeout-cleared outgoing Sensors, then publishes 76% to verify recovery. Registered as `battery_freshness_test` under colcon/ament GTest; no proto or application behavior change. AmurCore code unchanged in this cycle; peer plan updated for handoff. ROS2 Humble build/test and Qt5/loopback CI for these feature HEADs NOT VERIFIED; do not mark regression PASS until CI execution. P1 IN PROGRESS; P1-01..08 hardware/manual NOT RUN, P2 TODO. Next: inspect latest Actions runs/jobs and fix any test compilation/runtime errors, then add UI disconnect/offscreen coverage.


### P1 battery presentation + freshness regression follow-up (2026-11-08)
- AmurCore [3a80b038](https://github.com/ARDev1161/AmurCore/commit/3a80b038a1bcb36d3a10a6236acde3c009d8f8e2), [9d828a8e](https://github.com/ARDev1161/AmurCore/commit/9d828a8e5da4029e5db3bf688d4dfdf7720d3e79), [be7a30dc](https://github.com/ARDev1161/AmurCore/commit/be7a30dca44804562b13c74b4b370a41198e6bd3), [2654b486](https://github.com/ARDev1161/AmurCore/commit/2654b486d5941726d65598386492b85dce8b5bd0): extracted Qt battery display policy and added offscreen CTest for absent, 42%, charging, NaN, invalid percentages and absent again. No wire changes.
- rc_agent [a88bdc04](https://github.com/ARDev1161/rc_agent/commit/a88bdc042c18d731ec0905982a20378650ef197c) extends ROS2 battery freshness test with initial absence and post-recovery freshness checks.
- Source HEAD accepted by GitHub contents API. CI current HEAD NOT VERIFIED: GitHub connector exposes PR-triggered commit workflow runs only and combined status was empty; public Actions web fetch unavailable. Qt5/ROS2 local builds NOT RUN. Do not claim PASS. Next: inspect current feature-branch Actions job logs, fix any compile failures, and perform hardware P1-01..08 when robot available. P1 IN PROGRESS; P2 TODO. Documentation branches untouched, prior docs deploy success applies only to their documented SHAs.


### P1 battery concurrency and reconnect regression — 2026-11-08
- rc_agent [73f08412](https://github.com/ARDev1161/rc_agent/commit/73f0841247a6a0fb61173e799ac9f5430095503e) moves the 5-second freshness check and timestamp read under the same Sensors mutex as the ROS callback and gRPC snapshot, preventing a timestamp race under future multi-threaded executor configurations. No wire change.
- AmurCore [9a9b031b](https://github.com/ARDev1161/AmurCore/commit/9a9b031bc4343b9fd11863bc6ed13504f6ab770c) extends Qt offscreen battery presentation regression with absent→76% reconnect, 0% and 100% boundaries, then absent again; no production or proto changes.
- Source commits accepted by GitHub API. Local Qt/ROS2 toolchain tests NOT RUN; GitHub Actions HEAD results NOT VERIFIED: connector returns no commit statuses and public Actions pages are inaccessible. Do not infer green from prior SHA. P1 IN PROGRESS; manual P1-01..08 NOT RUN; P2 TODO. Next: inspect current Actions logs when accessible, fix any red jobs, then run simulator/live P1 gates. Docs branches unchanged.


### P1 battery regression extension — 2026-11-08
- rc_agent [`43d71c8e`](https://github.com/ARDev1161/rc_agent/commit/43d71c8e44a9f46121283d9b288f7d9a82ad2c7a) extends the real ROS2 publisher/subscriber/timer test to require a second expiration and third recovery (33%) after the previous 42%→timeout→76% cycle. This tests repeated timeout bookkeeping, not merely one-shot expiry.
- AmurCore [`d6e42fe6`](https://github.com/ARDev1161/AmurCore/commit/d6e42fe6e81c8af383be898cd58dcc97a359a65f) adds Qt offscreen presentation checks for positive and negative infinity, which must show unknown rather than fake charge.
- Local toolchains: colcon, ROS2 installation, qmake, and protoc unavailable; tests NOT RUN locally. GitHub commit-workflow query is PR-only and returns empty for feature HEAD, so Actions HEAD statuses NOT VERIFIED (not PASS). No protobuf/production changes, no docs branch changes. P1 IN PROGRESS, P1-01..08 hardware/live gates NOT RUN; P2 TODO. Next: obtain current feature-branch Actions job results and fix any failing build/test, then conduct simulator/live P1 acceptance.


### P1 battery refresh/status regression (2026-11-08)
- rc_agent [`e3f316f9`](https://github.com/ARDev1161/rc_agent/commit/e3f316f986452509a34202e384e37050939136f1) / [`3abd8285`](https://github.com/ARDev1161/rc_agent/commit/3abd8285ff34f504473eeaf79ed431a2e7dfc4a5) extend real ROS2 battery test with an in-window 55% refresh, ensuring it survives the previous sample's expiry tick. Test remains timing-sensitive on overloaded runners; investigate if flaky rather than weakening checks.
- AmurCore [`785c8c6f`](https://github.com/ARDev1161/AmurCore/commit/785c8c6fe68cb94e26a582de861346a5a0a7d25a) adds Qt offscreen FULL and UNKNOWN power-supply-status assertions: neither should incorrectly append charging to 100%.
- Shared proto/production code unchanged. No local Qt5/ROS2 toolchains; build and tests NOT RUN. Current feature HEAD Actions results NOT VERIFIED: available commit-workflow connector filters push runs. P1 IN PROGRESS, live P1-01..08 NOT RUN, P2 TODO. Next: inspect current Actions via browser/API when accessible, fix any failure; continue simulation gates, then hardware smoke tests.


### P1 cross-transport battery snapshot isolation (2026-11-08)
- AmurCore [`7afc6cbe`](https://github.com/ARDev1161/AmurCore/commit/7afc6cbeb8f65df64bcce5ad30759d9980fad6b0): expanded real gRPC unary loopback regression: a complete Sensors snapshot with no battery must clear the previous battery while retaining robot_info; a subsequent recovered 76% CHARGING sample must be accepted. No production/proto changes.
- rc_agent [`fb9f2b9a`](https://github.com/ARDev1161/rc_agent/commit/fb9f2b9afe35cfe311e88a0da830d6b73d7971d3): ROS2 BatteryState expiry/recovery regression now asserts repeated timeout clears only battery, not robot_info. No production/proto changes.
- Tests added, not executed locally (ROS2 Humble/Qt5/gRPC toolchains unavailable). GitHub commit workflow-run connector filters PR runs and cannot establish push CI status; current SHA CI NOT VERIFIED. P1 IN PROGRESS, hardware/live P1-01..08 NOT RUN; P2 TODO. Next: inspect feature-head Actions results and fix any failing jobs; continue deterministic transport/simulator P1 checks.


### P1 explicit absent battery handling (2026-11-08)
- rc_agent [`dda974db`](https://github.com/ARDev1161/rc_agent/commit/dda974dba3551c9bcfb3765bbf9af8ba64748fd4) clears optional `Sensors.batteryState` immediately on ROS `BatteryState.present=false`, rather than retaining the previous pack until the timeout. [`a4f7ead0`](https://github.com/ARDev1161/rc_agent/commit/a4f7ead0b4e99ddcea96098ab0193fca10b09790) adds real ROS publish/subscribe regression for explicit absence, identity preservation and recovery. AmurCore [`9ce89930`](https://github.com/ARDev1161/AmurCore/commit/9ce89930e2cc1e65c55016a0a0b0e7cbfcf7ee84) adds legacy-wire gRPC regression for a present=false field and identity preservation. No shared proto changes; production docs branch untouched.
- CI NOT VERIFIED: connector's commit-run lookup only lists PR-triggered workflows, and public API retrieval unavailable. Local ROS2/Qt5 build NOT RUN. Tests added, not claimed PASS. P1 IN PROGRESS; physical P1-01..08 NOT RUN; P2 TODO. Next: inspect Actions current feature SHA using GitHub UI/API when available, fix failures, and continue deterministic acceptance coverage.


### P1 battery test timing reliability (2026-11-08)
- rc_agent [`9b4aad5b`](https://github.com/ARDev1161/rc_agent/commit/9b4aad5b3816400c72fdce1bfeafeb72d35f41c5) widens ROS2 `BatteryFreshness` test's receive timeout from 350 ms to 1200 ms and checks refresh after 650+700 ms (crosses original expiration with 500 ms slack after refresh), reducing scheduler-jitter false failures without weakening the expiry/recovery assertions. No production or protobuf change; AmurCore source unchanged.
- Validation: source-level timing arithmetic checked (650+700 > 1200, 700 < 1200); local ROS2/Qt toolchains unavailable, **colcon/Qt CI current feature HEAD NOT VERIFIED** because connector returns PR-only workflow runs and public API inaccessible. Earlier green workflow SHA is not evidence for this HEAD. Hardware/VPN/joystick/Nav2/map/GStreamer/old↔new live P1 gates NOT RUN. P1 IN PROGRESS, P2 TODO. Next: obtain current feature-head Actions logs, resolve failures, execute manual checklist once robot available.


### P1 CI trigger coverage correction — 2026-11-08
- Corrected `.github/workflows/rc-api-v2.yml` push path filters in BOTH repositories to include `**/*.py`, `**/*.sh` and `**/*.cc`, in addition to existing C++/header/proto/CMake/UI/config paths. Previously a Python-only contract regression could land without running the mandatory workflow; this is now covered. AmurCore workflow change [`dafd21ee`](https://github.com/ARDev1161/AmurCore/commit/dafd21eeb5b473d45f4c1050b91a7f9de2af6778); rc_agent [`b466f7fd`](https://github.com/ARDev1161/rc_agent/commit/b466f7fda510d58ee0bae4b9996e3ffaa0c72111).
- Validation: source diff inspected; current HEAD Actions results **NOT VERIFIED** pending actual run/job inspection. Workflow-only changes should trigger a new push run via existing `.github/workflows/rc-api-v2.yml` path filter. No shared protobuf or production code changed. P1 IN PROGRESS; live P1-01..08 NOT RUN; P2 TODO. Next: inspect new push run for both HEADs, fix failures, and perform physical acceptance when robot available.


## P1 video pipeline shared-snapshot mutex regression — 2026-11-08
- Source audit discovered `main.cpp::StreamPipelineReceiver::onPipeline()` writing to shared `Sensors.video_stream.pipeline` without the gRPC sender's `muClient`, racing with `grpcClient::DataExchange()` and `DataStreamExchange()` protobuf snapshot copies on another thread. The existing `VideoControlNode::updateStatus()` and optional `BatteryStateNode` already use the gRPC mutex.
- Extracted the unchanged ROS topic receiver behavior to `modules/video/stream_pipeline_receiver.{h,cpp}`, taking the exact `grpcClient::getMutex()` and protecting duplicate check+mutation. Constructed receiver only after network client initialization so the real mutex is available; startup pipeline from config remains unchanged.
- Added actual ROS2 publisher/subscriber `video_pipeline_snapshot_test` under Humble colcon: no fabricated pipeline before publication; normal/long pipeline update; empty message ignored; identity+battery preserved; a locked sender snapshot is not modified by ROS callback, and update arrives after releasing mutex. Explicit `std_msgs` CMake/package dependency. No GStreamer pipeline start/stop behavior, video source selection or P5 inactive-robot gating modified.
- Paired AmurCore real-peer test expected to assert legacy unary video pipeline snapshot and update reach operator correctly. **CI at new source HEAD NOT YET VERIFIED**. No P2 features, no proto field changes, physical/VPN/Nav2/joystick/GStreamer manual P1-01..08 NOT RUN.


## P1 video pipeline snapshot mutex CI — VERIFIED 2026-11-08
- Tested feature source commit `8a7dffe9da07caf121d3307d2307652d9fcd0c48`: [RC API v2 CI #42503503640](https://github.com/ARDev1161/rc_agent/actions/runs/42503503640) **SUCCESS**, all 5 jobs at exactly this code SHA; ROS2 Humble colcon output confirms `VideoPipelineSnapshot.RosCallbackUsesGrpcSensorsMutex` PASS (398 ms), plus existing battery freshness, gRPC mocks, real-peer and protobuf/config tests.
- Paired tested peer SHA `348f4cdde1ce7ea0d24e1e1468e24a18f5902aa1`; shared protobuf files and legacy fields/RPC IDs unchanged.
- Docs-only `documentation` SHA `c47b43eeab40b044568ea2a48ca8d8f3251bc580`: [Sphinx + Doxygen + Pages #42503817105](https://github.com/ARDev1161/rc_agent/actions/runs/42503817105) **SUCCESS**, including publishing. Doc branch remains isolated from C++ branches.
- This handoff is PLAN.md-only; the preceding source SHA is the verified code, not this successor commit. No P2, no merge to dev/master.
- **P1 IN PROGRESS:** real robot/VPN/joystick/Qt operator interactions/Nav2/OccupancyGrid visualization/live GStreamer and old↔new live manual checks **NOT RUN**, P1-01–P1-08 outstanding. The test validates video pipeline metadata synchronization, NOT actual capture/playback or P5 video gating. Next: manual simulator/robot acceptance; if unavailable, only work on specifically evidenced P1 correctness defects, not P2.


### P1 handoff — 2026-11-08 clean stream EOF regression
- rc_agent feature commits: [f93039b0](https://github.com/ARDev1161/rc_agent/commit/f93039b0bea13fceae8b73495f45f3335af19cbb) reconnects when a peer ends a bidirectional stream with OK and retry is enabled; [0c10f902](https://github.com/ARDev1161/rc_agent/commit/0c10f902170a1f2331cfb6f74f58c72bb1862b9d) adds deterministic loopback coverage for a fresh RobotInfo after clean EOF and explicit stop. No shared proto changes; AmurCore production sources unchanged.
- CI for these new commits: PENDING/NOT YET VERIFIED (historical green runs are not evidence for these SHA). Locally ROS2/Qt5 builds NOT RUN in this environment. Verify current feature-head workflow jobs and fix failures before declaring this regression complete.
- Phase remains P1 IN PROGRESS. P1-01..08 hardware/VPN/joystick/Nav2/map/GStreamer/legacy live smoke still OPEN; P2 blocked by P1 acceptance. Next: check rc_agent current HEAD CI, then test stream reconnect and snapshot with real robot when available.


### P1 idle MapStream cancellation regression — 2026-11-09
- rc_agent [53173cf8](https://github.com/ARDev1161/rc_agent/commit/53173cf853d68791f5da4446387d459661ed7042) extends the real gRPC loopback client test: mock operator accepts an initial map snapshot and then leaves the RPC idle; explicit stopStream must unblock the idle client, return CANCELLED, and not require a map_seq update. This is an idle shutdown regression, **not** proof that peer clean EOF is detected without a subsequent change.
- AmurCore production code and shared protobuf are unchanged. P1 remains IN PROGRESS; P1-01..08 physical/VPN/GUI/Nav2/GStreamer/legacy live checks NOT RUN, P2 TODO.
- Test authored, local Qt5/ROS2/gRPC build NOT RUN (toolchain unavailable). CI for new rc_agent test commit and subsequent PLAN-only HEAD NOT VERIFIED until current Actions jobs complete. Historical successful runs: rc_agent #42519848151 (5/5), AmurCore #42503577387 (3/3), on older source SHAs.
- Next: inspect new rc_agent feature SHA CI, repair any red jobs, then extend a deterministic idle peer-disconnect test without periodically transmitting unchanged maps. Do not change stream semantics on inference alone.


### P1 map peer-EOF reconnect regression — 2026-11-09
- rc_agent test commit [0ea5cd06](https://github.com/ARDev1161/rc_agent/commit/0ea5cd060b463f566b80f49530268a8f5577fafb) adds real gRPC loopback coverage: server returns clean OK after initial map snapshot; idle map must not retransmit; after map_seq changes, client must detect EOF, reconnect and deliver fresh map snapshot; explicit stop must return CANCELLED. No production code or shared protobuf changes. AmurCore production sources unchanged.
- Prior rc_agent [53173cf8](https://github.com/ARDev1161/rc_agent/commit/53173cf853d68791f5da4446387d459661ed7042) test CI [run 42539929484](https://github.com/ARDev1161/rc_agent/actions/runs/42539929484) SUCCESS 5/5 jobs; this does NOT validate new 0ea5cd06 regression. New CI PENDING/NOT VERIFIED at handoff; local Qt5/ROS2 build NOT RUN (toolchain unavailable). AmurCore last verified source CI [42503577387](https://github.com/ARDev1161/AmurCore/actions/runs/42503577387), 3/3 SUCCESS on older SHA.
- P1 IN PROGRESS. P1-01..08 real robot/VPN/joystick/Nav2/map/GStreamer/old-new live gates NOT RUN; P2 remains TODO. Next: check new rc_agent test run and fix any failures; if green, investigate idle peer EOF detection latency separately without transmitting unchanged maps. No source/doc branch merges.


### P1 idle pose/zone stream cancellation regression — 2026-11-09
- rc_agent test commit [8973d66c](https://github.com/ARDev1161/rc_agent/commit/8973d66cf821e1154ff2736a2c2d742946f7f7d9) extends real gRPC loopback to PoseStream and ZoneStream: initial snapshot, no unchanged retransmission during idle, stopStream cancels idle RPC and returns CANCELLED. Complements MapStream coverage. No production/protobuf changes; AmurCore sources unchanged.
- CI at new test SHA NOT VERIFIED at authorship; local Qt5/ROS2/gRPC toolchain tests NOT RUN. Previously verified rc_agent [run 42545355642](https://github.com/ARDev1161/rc_agent/actions/runs/42545355642) 5/5 SUCCESS on older 0ea5cd06 test commit; AmurCore [run 42503577387](https://github.com/ARDev1161/AmurCore/actions/runs/42503577387) 3/3 SUCCESS on older 348f4cdd source SHA. Do not treat these as current HEAD CI.
- P1 IN PROGRESS; physical P1-01..08 NOT RUN, P2 TODO. Next: inspect current rc_agent feature CI and repair failures; then investigate clean peer EOF while no map/pose/zone updates, without sending unchanged payloads. Documentation branches untouched.


### P1 PoseStream clean-EOF reconnect regression — 2026-11-09
- rc_agent [81008360](https://github.com/ARDev1161/rc_agent/commit/81008360e1d2426227ff70e13d8cc2448f1777cc) extends actual gRPC loopback mock with one-shot clean OK server EOF on PoseStream; asserts initial snapshot, no retransmission while pose unchanged, detection of closed RPC on subsequent position change, reconnect and new snapshot, and terminal CANCELLED on explicit stop. Complements prior MapStream and idle Pose/Zone cancellation tests. No production/proto changes; AmurCore production unchanged.
- Validation: GitHub accepted source commit; local Qt5/ROS2/gRPC tests NOT RUN (toolchain unavailable). Current HEAD Actions status NOT VERIFIED: GitHub connector lists only PR-triggered runs and web Actions page inaccessible. Prior successful runs on older SHA do not validate this test. No manual robot/VPN/UI/Nav2/GStreamer/legacy live P1-01..08 checks run. P1 IN PROGRESS, P2 TODO. Next: inspect feature HEAD CI via GitHub Actions when accessible; fix red jobs, then test ZoneStream clean EOF on changed zone_seq and investigate idle peer EOF latency without unchanged payload retransmissions.


### P1 ZoneStream clean-EOF reconnect regression — 2026-11-09
- rc_agent source [701811f7](https://github.com/ARDev1161/rc_agent/commit/701811f71f734f8303371f3f9f34a207478d8cb3): actual C++ gRPC loopback mock closes ZoneStream with clean OK after first snapshot; unchanged zone is not retransmitted; incrementing zone_seq must detect EOF and reconnect with a fresh snapshot; explicit stop returns CANCELLED. No production/protobuf changes; AmurCore source unchanged.
- Previous PR CI: rc_agent [run 42568700340](https://github.com/ARDev1161/rc_agent/actions/runs/42568700340) **5/5 SUCCESS** at prior feature HEAD 2329bf13, NOT evidence for new 701811f7. AmurCore [run 42568697631](https://github.com/ARDev1161/AmurCore/actions/runs/42568697631) protobuf and real-peer SUCCESS, qt-app-build still IN_PROGRESS when checked at previous AmurCore HEAD ccde38e4. New HEAD CI PENDING; local toolchain tests NOT RUN.
- Draft PRs for discoverable PR-triggered Actions: [AmurCore #5](https://github.com/ARDev1161/AmurCore/pull/5), [rc_agent #4](https://github.com/ARDev1161/rc_agent/pull/4). fetch_commit_workflow_runs finds PR runs by exact head SHA, then fetch_workflow_run_jobs and fetch_workflow_job_logs. Do not infer push CI from empty results.
- P1 IN PROGRESS, manual P1-01..08 NOT RUN (robot repair), P2 TODO. Next: verify new test HEAD CI, fix failures; investigate idle EOF latency without duplicate telemetry or crossing phase gates. Documentation branches untouched.


### P1 ZoneStream payload freshness and current-head CI — 2026-11-09
- Verified exact feature HEAD before this change: rc_agent `90c7e3226fe46ce13ac012bc02c43698044cb3c1`, [PR Actions #42568912055](https://github.com/ARDev1161/rc_agent/actions/runs/42568912055) **SUCCESS 5/5**; AmurCore `0ff3535417d94756176207a4f624d470e28a651c`, [PR Actions #42568916582](https://github.com/ARDev1161/AmurCore/actions/runs/42568916582) **SUCCESS 3/3**. Both are software CI only, not hardware acceptance.
- New rc_agent source [16779b66](https://github.com/ARDev1161/rc_agent/commit/16779b66d332aeea89013069976e431363dfe4dc) strengthens ZoneStream clean-EOF reconnect test: mutate ZoneMap zone ID with zone_seq under the map mutex, capture the actual received zone ID on mock server and require fresh zone payload (not just another RPC). No production or shared proto changes, AmurCore source unchanged. New test CI NOT YET VERIFIED at authorship; local toolchain NOT RUN.
- P1 IN PROGRESS; manual P1-01..08 NOT RUN (robot under repair), P2 TODO. Next: verify current PR CI by exact SHA, fix any failures; investigate idle EOF detection as a separate behavior without sending duplicate unchanged snapshots. No merges or documentation branch edits.


### P1 real-peer callback mutex contention regression — 2026-11-09
- rc_agent HEAD `3c75314fcc2f1060274de5e2c1ddfd3f7a7fdb39`: [Actions #42569579882](https://github.com/ARDev1161/rc_agent/actions/runs/42569579882) SUCCESS 5/5, including new ZoneMap payload freshness loopback.
- AmurCore HEAD `7c5ac5b61ed31b35a6f8b5867068e7c7db0492c7`: [Actions #42569586475](https://github.com/ARDev1161/AmurCore/actions/runs/42569586475) FAILURE 2/3; `qt-app-build` and `protobuf-compatibility` SUCCESS; real-peer integration FAILED `Stopped peer callback held UI mutex`. The original callback's single `try_lock()` can spuriously fail when another RPC holds the shared mutex, even though the callback's own RPC has released it. Production server code already unlocks before notify.
- AmurCore source test fix [0fc2c666](https://github.com/ARDev1161/AmurCore/commit/0fc2c666955d509f6986ed5413c612d1b0ee9c88) replaces single-shot try_lock with bounded 250ms asynchronous mutex availability probe, distinguishing transient concurrent contention from a callback that still holds the mutex. No production, proto, or rc_agent source changed. This new HEAD CI NOT VERIFIED at authorship; do not claim green until exact HEAD passes.
- P1 IN PROGRESS, manual P1-01..08 NOT RUN, P2 TODO. Next: inspect new AmurCore PR CI for real-peer test; if failure persists, inspect callback and RPC locking with concrete log evidence. Docs branches untouched, no merges.


### P1 real-peer optional BatteryState snapshot regression — 2026-11-09
- Exact preceding feature HEAD CI: AmurCore `d785aaa4b00f9cf576ca81c484abf88448c42428` [run 42572728961](https://github.com/ARDev1161/AmurCore/actions/runs/42572728961) SUCCESS 3/3 (real peer, full Qt5, protobuf); rc_agent `c64693ece1aef54136e8c1666c38d02e0cfeb710` [run 42572733403](https://github.com/ARDev1161/rc_agent/actions/runs/42572733403) SUCCESS 5/5 (ROS2 Humble, gRPC loopback, config, protobuf, real peer).
- AmurCore test source [0a4646cb](https://github.com/ARDev1161/AmurCore/commit/0a4646cb5a07f5d5f75c71df5eb5586da2a2ccd0) adds real AmurCore↔rc_agent unary regression: after a battery-bearing v2 snapshot, clear optional BatteryState, require receiver clears stale battery while preserving robot_id/api_major; verify callback and subsequent stream count. No production/proto changes. rc_agent source unchanged; its existing optional battery tests remain in place.
- New HEAD CI pending verification. Local Qt5/ROS2 toolchains NOT RUN. P1 IN PROGRESS; manual P1-01..08 NOT RUN (robot under repair); P2 TODO. Next: inspect current PR CI jobs/logs; fix red if any. Docs branches separate and unchanged, no merges.


### P1 optional BatteryState symmetry — 2026-11-09
- Before this change exact feature HEADs were AmurCore `758959872a3b1eeffab60c492e5f160ccbf95eb4` ([Actions #42580267193](https://github.com/ARDev1161/AmurCore/actions/runs/42580267193), SUCCESS 3/3) and rc_agent `260913a4e2cc979badcca511a8b9a4e222f25d33` ([Actions #42580275887](https://github.com/ARDev1161/rc_agent/actions/runs/42580275887), SUCCESS 5/5). Both verified at exact feature HEAD; hardware NOT RUN.
- rc_agent [ba0ca143](https://github.com/ARDev1161/rc_agent/commit/ba0ca143a1f0732fae0440c60e3e6ab2746fd801) adds real gRPC unary loopback regression for a transition from present BatteryState to absent BatteryState: mock receiver must clear optional field and retain robot_id/capability/velocity_axes. This complements AmurCore real-peer test of battery disappearance. No production code, protobuf, ROS endpoint or branch architecture changed.
- New source CI NOT VERIFIED at authorship; local ROS2/Qt5 toolchains NOT RUN. P1 IN PROGRESS, P1-01..08 physical/VPN/GUI/Nav2/GStreamer/legacy live checks NOT RUN, P2 TODO. Next: verify both current feature HEADs using Draft PR Actions, fix red jobs if any; continue only evidenced P1 correctness regressions. No merge; documentation remains separate.
