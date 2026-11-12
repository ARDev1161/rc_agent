# Validation and migration gates

The canonical [P0–P6 roadmap](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/PLAN.md) defines acceptance. This site describes the **documented P1 checkpoint**, not a claim of full v2 rollout.

| Phase | State | Gate |
| --- | --- | --- |
| P0 | Done | branches and architecture frozen |
| P1 | **In progress** | protobuf mirror, old/new wire, Qt/ROS2, loopback, real robot/manual smoke |
| P2 | **In progress** (authorized overlap; P1 manual pending) | trusted local ROS2 endpoint profiles, two robot profiles, real Nav2 result/cancellation tests |
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


## P1 concurrent shutdown and retry-backoff cancellation — 2026-11-08

The blocked unary RPC cancellation fix and test passed on source revision [`b05bab6d`](https://github.com/ARDev1161/rc_agent/commit/b05bab6d1340d3de4ed9bd1bd63ef460f327b27b), [Actions #42450762717](https://github.com/ARDev1161/rc_agent/actions/runs/42450762717), **5/5 jobs SUCCESS**. Paired AmurCore source [`60a6dd9e`](https://github.com/ARDev1161/AmurCore/commit/60a6dd9e7f3261deb5a73ef1a539af7fac0a2f64) passed [Actions #42450768529](https://github.com/ARDev1161/AmurCore/actions/runs/42450768529), **3/3 jobs SUCCESS**.

Newer transport changes [`453be21f`](https://github.com/ARDev1161/rc_agent/commit/453be21f643797a8450399c806898a9f7454c37f) and [`4cb1faa9`](https://github.com/ARDev1161/rc_agent/commit/4cb1faa9e0cc77e8ebbece37e44357413683bf40) protect against late stream controls and test concurrent blocked unary+stream calls on one client. [`1a0f0a50`](https://github.com/ARDev1161/rc_agent/commit/1a0f0a50930db89a5e0117bfedb74a717d8df129) and [`fc28ea70`](https://github.com/ARDev1161/rc_agent/commit/fc28ea7083c9af2efb7d3c50e954470e999c9b6c) make explicit stop during stream retry backoff return CANCELLED and add a mock regression. CI for these **newer commits is NOT VERIFIED**. A gRPC cancellation is not a physical stop or P5 deadman/lease/watchdog guarantee. P1-01–P1-08 manual gates remain **NOT RUN**, P1 IN PROGRESS, P2 TODO.


## P1 video metadata ROS2 callback / gRPC snapshot safety — 2026-11-08

An existing ROS2 `/gst/pipeline` subscriber was writing directly into the outgoing `Robot::Sensors.video_stream.pipeline` without the mutex used by the gRPC sender, creating a data race with telemetry snapshots. The existing receiver is now a small standalone [`StreamPipelineReceiver`](https://github.com/ARDev1161/rc_agent/blob/feat/rc-api-v2/modules/video/stream_pipeline_receiver.h) node whose constructor receives the same `grpcClient::getMutex()`; the duplicate check and protobuf mutation are protected by this mutex. It starts after `runClient()` has created that mutex.

An actual ROS2 pub/sub `video_pipeline_snapshot_test` checks the initial absence, normal and long pipeline updates, empty message ignored, preservation of robot identity/battery and a sender mutex held while the callback is pending. The counterpart real `AmurCore` gRPC integration checks legacy/v2 video metadata snapshots. [Tested source `8a7dffe9da07`](https://github.com/ARDev1161/rc_agent/commit/8a7dffe9da07caf121d3307d2307652d9fcd0c48) — [CI 42503503640](https://github.com/ARDev1161/rc_agent/actions/runs/42503503640), **all five jobs SUCCESS**, including ROS2 Humble colcon, config, protobuf and real-peer tests.

This is protobuf metadata synchronization only; it does **not** test an actual camera/GStreamer pipeline, video playback, stream gating or physical robot. Hardware/manual P1-01–P1-08 remain NOT RUN and P1 IN PROGRESS.

## P2 auxiliary velocity profile binding — 2026-11-13

The gateway binds the legacy `cmd_vel`, `cmd_vel_smoothed` and `cmd_vel_unsafety` selectors only to trusted local ROS2 endpoints from the libconfig robot profile. `cmd_vel_smoothed_topic` and `cmd_vel_nav_topic` are now exposed in the gateway's `Amur.API.endpoints` settings and passed to the ROS2 adapter; omitted values retain legacy defaults. Two independent config files and invalid-type/empty values are covered by gateway C++ GTests. This does **not** implement v2 velocity authorization.

[Functional rc_agent test revision `15327dc0`](https://github.com/ARDev1161/rc_agent/commit/15327dc0eebd7b36294c742177d19f8607bb0692) · [source CI #43327905089](https://github.com/ARDev1161/rc_agent/actions/runs/43327905089) (verify completion for this revision). P1 software/manual and physical gates remain open.


## P2 full Nav2 action/initialpose profile — 2026-11-13

The trusted `Amur.API.endpoints` libconfig mapping now resolves all four existing Nav2 action names (`navigate_to_pose`, `navigate_through_poses`, `follow_waypoints`, `spin`) and the `initial_pose_topic` together in the production gateway startup path. New C++ GTests load two distinct on-disk robot profiles, assert all names, retain legacy defaults when omitted, and reject wrong-type/empty values. Existing ROS2 Humble mock Nav2 action tests separately exercise goal/result and isolated cancellation routing.

[rc_agent functional test commit `3cde859b`](https://github.com/ARDev1161/rc_agent/commit/3cde859bfd2a35f9e47371e7ebcab64ae1d9cde0) · [source CI #43329770953](https://github.com/ARDev1161/rc_agent/actions/runs/43329770953) (check actual result; not presumed green). This **does not** establish real TurtleBot3/physical Nav2 acceptance, P3 multi-robot gRPC sessions, or enabled v2 velocity control.
