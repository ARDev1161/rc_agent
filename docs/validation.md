# Validation and migration gates

The canonical [P0–P6 roadmap](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/PLAN.md) defines acceptance. This site describes the **documented P1 checkpoint**, not a claim of full v2 rollout.

| Phase | State | Gate |
| --- | --- | --- |
| P0 | Done | branches and architecture frozen |
| P1 | **In progress** | protobuf mirror, old/new wire, Qt/ROS2, loopback, real robot/manual smoke |
| P2 | Not started | configurable trusted ROS2 endpoints |
| P3 | Not started | independent multi-robot sessions |
| P4 | Not started | tasks/dialogue adapter |
| P5 | Not started | change-only streams and safe teleop |
| P6 | Not started | end-to-end verification |

Historical successful Actions: [AmurCore CI #20](https://github.com/ARDev1161/AmurCore/actions/runs/41262962581) and [rc_agent CI #25](https://github.com/ARDev1161/rc_agent/actions/runs/41263125743). These runs are **not** evidence for all future commits or real hardware.

The [eight-case P1 manual checklist](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/docs/P1_MANUAL_ACCEPTANCE.md) must still verify RobotInfo, legacy clients in both directions, joystick with stop, map/pose/Nav2, video start/stop, idle responsiveness and reconnect. Do not mark P1 DONE on docs or protocol-only CI alone.

## Scope of this documentation CI

It checks documentation branch isolation, Doxygen XML/HTML API generation from a separately checked out source revision, successful Sphinx rendering, Mermaid markup, navigable API pages, and presence of a source SHA in the site title. It does **not** replace the source code's Qt5, ROS2, gRPC or hardware test matrix.

## P1 mock deadlock regression — 2026-11-08

[Source revision da2b081dbe25a4bf8c560cfa3db7eb0f0ac18252](https://github.com/ARDev1161/rc_agent/commit/da2b081dbe25a4bf8c560cfa3db7eb0f0ac18252) fixes `OperatorMock::capture()` recursively calling itself under the descriptor mutex. A red [prior run](https://github.com/ARDev1161/rc_agent/actions/runs/42429638780) hung after `Client started`. The test now captures robot ID/capability/axis under one lock for unary and streamed snapshots, clears those fields on legacy snapshots, and tests a v2-to-legacy transition.

This is a simulated transport regression, **not a completed P1 manual gate**. [CI run](https://github.com/ARDev1161/rc_agent/actions/runs/42431794847) must pass before the fix is verified. The actual robot/VPN/joystick/Nav2/map/GStreamer checks remain outstanding.


## Real AmurCore ↔ rc_agent P1 integration test — 2026-11-08

The independent working branches now run a new **real-peer-grpc-loopback** CI job. Each job checks out the two actual C++ implementations, compares the mirrored protobuf contracts, compiles the production transport peers together, and exercises an ephemeral localhost gRPC connection. The shared scenario, maintained under [AmurCore/tests/real_peer_smoke.cpp](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/tests/real_peer_smoke.cpp), checks legacy control/sensor roundtrips, v2 descriptor and supported axes, legacy-after-v2 reset, fresh-channel reconnect, and workstation mutex availability during callbacks.

The [current CI workflow](https://github.com/ARDev1161/rc_agent/actions/runs/42432813251) must complete successfully before this gate can be marked PASS. No test sends commands to robot actuators. Hardware/VPN/joystick/Nav2/map/GStreamer acceptance remains separate and **not completed**; P1 is still IN PROGRESS.


### Real-peer acceptance result (CI, 2026-11-08)

The cross-repository test passed on the exact source revision [`41db27715aa2e453742fb712b548b7bcdfe51dd2`](https://github.com/ARDev1161/rc_agent/commit/41db27715aa2e453742fb712b548b7bcdfe51dd2): [workflow #42432945859](https://github.com/ARDev1161/rc_agent/actions/runs/42432945859) **SUCCESS**. Both repositories now compile and run the actual workstation gRPC server with the actual gateway gRPC client in CI. This is a headless P1 transport acceptance check, **not** P1-01 through P1-08 real-robot/UI/Nav2/GStreamer acceptance; P1 remains in progress.


## Bounded reconnection and one-shot RPC failures — 2026-11-08

The P1 gateway transport now respects `tryConnectIfFailed` in legacy unary and streaming RPCs. Unary requests copy the telemetry snapshot under the ROS mutex and send it with a three-second per-attempt deadline. Failed unary replies do not overwrite prior controls; retry backoff does not hold ROS callback mutex. `NetworkController` forwards the caller's retry setting, and the stream stop flag is atomic. The new mock loopback regression exercises one-shot `UNAVAILABLE`, successful recovery from a transient failure and unlocked mutex during retries.

[Source tested at **29cda7bf4a39**](https://github.com/ARDev1161/rc_agent/commit/29cda7bf4a390bb5f672feacdbcd4f70d880588a) — [CI run](https://github.com/ARDev1161/rc_agent/actions/runs/42435001800) **SUCCESS** (all five jobs, including real peer and ROS2 Humble).

**Safety boundary:** a canceled gRPC stream does not itself stop physical actuators. P5 teleop watchdog, lease/session and deadman protections remain out of P1.


## P1: canceling a blocked legacy gRPC stream (2026-11-08)

`grpcClient::stopStream()` now marks an instance terminal and invokes `TryCancel()` for each registered live gRPC `ClientContext`. Registration and removal are mutex protected; it applies to data, map, pose and zone streaming. Retry backoff checks for shutdown in short intervals. Reconnection after an explicit stop creates a **new** gateway client instance.

The [new mock integration test](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/tests/grpc_client_smoke.cpp) withholds a response and verifies the client exits the blocking read, reports cancellation, retains the previous controls and cannot silently restart. The paired real-server test is under [AmurCore](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/tests/real_peer_smoke.cpp). [Source revision `598c6da8efd4`](https://github.com/ARDev1161/rc_agent/commit/598c6da8efd42bd0de120c7947f2c2a776677019), [current CI](https://github.com/ARDev1161/rc_agent/actions/runs/42441827514) — check completion before marking PASS.

This is *transport lifecycle* only. It does not guarantee a stopped physical robot or implement the future P5 deadman/lease/watchdog.


## P1 coherent map snapshots without network I/O under mutex — 2026-11-08

The legacy `grpcClient::MapStream()` in [rc_agent](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/network/client.cpp) now snapshots the map sequence and `GetMapResponse` payload under one `muMap` lock and performs gRPC `Write`/`Read` **outside** the lock. This removes a potential ROS2 map callback stall and sequence/payload race. As before, the agent compares only `map_seq`, **not full OccupancyGrid contents**. The Map, Pose and Zone stream loops now respect failed gRPC Write/Read to let the legacy retry policy handle disconnections.

The cross-repo real-peer CI test verifies the initial map and a changed map sequence with independent objects, then cancellation. [Source 403d499735f5](https://github.com/ARDev1161/rc_agent/commit/403d499735f5b996ee86011c0ee459569d05de11) · [CI 42444234818](https://github.com/ARDev1161/rc_agent/actions/runs/42444234818). Verify this CI's result before claiming success. P1 manual hardware and UI gates remain outstanding; P5 changed-only telemetry beyond the legacy map sequence and video-selection policy have not been implemented.


## P1 consistent zone snapshots and independent pose/zone transport — 2026-11-08

`ZoneStream` now reads `zone_seq` and the corresponding zone map under one mutex acquisition and sends its snapshot without holding the mutex. Both `PoseStream` and `ZoneStream` construct their gRPC streams only after releasing the `muMap` lock. The paired [AmurCore real-peer test](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/tests/real_peer_smoke.cpp) verifies both legacy streams separately from the stopped `MapStream`, including changed data and cancellation.

[Working source f2ad035d2139](https://github.com/ARDev1161/rc_agent/commit/f2ad035d2139a6a0b534ef2b1ea07f2aec5c5c6e) / [CI 42445880201](https://github.com/ARDev1161/rc_agent/actions/runs/42445880201) (verify final result before claiming PASS). No proto change, new endpoint, Nav2 behavior or motor/velocity safety implementation. P1 remains IN PROGRESS pending manual P1-01–08.
