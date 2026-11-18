#include "network/v2_velocity_gate.h"
#include <chrono>
#include <stdexcept>
#include <limits>
#include <thread>
using rc::api::v2::VelocityGate;
using rc::api::v2::Velocity;
using namespace std::chrono_literals;
static void check(bool ok, const char* msg) { if (!ok) throw std::runtime_error(msg); }
static Velocity command(unsigned seq = 1) {
    Velocity v;
    v.set_sequence(seq);
    v.set_valid_for_ms(100);
    v.set_frame_id("base_link");
    v.mutable_linear()->set_x(0.2);
    return v;
}
int main() {
    VelocityGate gate;
    const auto t = VelocityGate::Clock::time_point{} + 10s;
    check(!gate.admit(command(), "s", "l", true, t), "accepted without lease");
    gate.grant("s", "l", t, 1000ms);
    check(!gate.admit(command(), "wrong", "l", true, t), "wrong session accepted");
    gate.grant("s", "l", t, 1000ms);
    check(!gate.admit(command(), "s", "wrong", true, t), "wrong lease accepted");
    gate.grant("s", "l", t, 1000ms);
    check(!gate.admit(command(), "s", "l", false, t), "deadman bypass");
    gate.grant("s", "l", t, 1000ms);
    check(gate.admit(command(), "s", "l", true, t), "valid command rejected");
    check(!gate.stopPending(t + 99ms), "premature watchdog stop");
    check(gate.stopPending(t + 100ms), "watchdog stop missing");
    check(gate.stopPending(t + 101ms), "unacknowledged stop lost");
    gate.grant("next", "l2", t + 102ms, 1000ms);
    check(!gate.admit(command(), "next", "l2", true, t + 102ms), "pending stop bypassed");
    gate.acknowledgeStopPublished();
    gate.grant("next", "l2", t + 103ms, 1000ms);
    check(gate.admit(command(), "next", "l2", true, t + 103ms), "fresh lease rejected");
    check(!gate.admit(command(), "next", "l2", true, t + 104ms), "sequence replay accepted");
    check(gate.stopPending(t + 104ms), "replay did not revoke motion");
    gate.acknowledgeStopPublished();
    gate.grant("disconnect", "l3", t + 200ms, 1000ms);
    check(gate.admit(command(), "disconnect", "l3", true, t + 200ms), "setup failed");
    gate.revoke();
    check(gate.stopPending(t + 201ms), "disconnect stop missing");
    check(gate.stopPending(t + 500ms), "stop lost before DDS acknowledgement");
    check(!gate.admit(command(2), "disconnect", "l3", true, t + 201ms), "stale session accepted");
    gate.acknowledgeStopPublished();
    check(!gate.stopPending(t + 501ms), "acknowledged stop still pending");
    // Lease expiry is independent of each command TTL and cannot be extended by traffic.
    gate.grant("short", "lease", t + 550ms, 50ms);
    check(gate.admit(command(), "short", "lease", true, t + 550ms), "short lease setup failed");
    check(gate.stopPending(t + 600ms), "lease expiration did not stop motion");
    gate.acknowledgeStopPublished();
    gate.grant("invalid", "lease", t + 601ms, 0ms);
    check(!gate.admit(command(), "invalid", "lease", true, t + 601ms), "zero-duration lease accepted");
    // Boundary conditions: fail closed on invalid numerical payloads and clocks.
    gate.grant("numeric", "lease", t + 600ms, 1000ms);
    auto nan_velocity = command();
    nan_velocity.mutable_linear()->set_x(std::numeric_limits<double>::quiet_NaN());
    check(!gate.admit(nan_velocity, "numeric", "lease", true, t + 600ms), "NaN linear velocity accepted");
    gate.grant("numeric", "lease", t + 601ms, 1000ms);
    auto inf_velocity = command();
    inf_velocity.mutable_angular()->set_z(std::numeric_limits<double>::infinity());
    check(!gate.admit(inf_velocity, "numeric", "lease", true, t + 601ms), "infinite angular velocity accepted");
    gate.grant("numeric", "lease", t + 602ms, 1000ms);
    auto long_ttl = command();
    long_ttl.set_valid_for_ms(1001);
    check(!gate.admit(long_ttl, "numeric", "lease", true, t + 602ms), "oversized TTL accepted");
    gate.grant("numeric", "lease", t + 603ms, 1000ms);
    auto no_frame = command();
    no_frame.clear_frame_id();
    check(!gate.admit(no_frame, "numeric", "lease", true, t + 603ms), "missing frame accepted");
    gate.grant("numeric", "lease", t + 604ms, 1000ms);
    check(!gate.admit(command(), "numeric", "lease", true, t + 603ms), "clock before grant accepted");
    gate.grant("numeric", "lease", t + 605ms, 1000ms);
    check(gate.admit(command(), "numeric", "lease", true, t + 605ms), "boundary setup failed");
    check(!gate.admit(command(2), "numeric", "lease", true, t + 604ms), "rewound clock accepted");
    check(gate.stopPending(t + 606ms), "rewound clock did not latch stop");
    gate.acknowledgeStopPublished();

    // A watchdog tick itself must fail closed if monotonic time regresses,
    // even when no subsequent network command arrives.
    gate.grant("watchdog-rollback", "lease", t + 700ms, 1000ms);
    check(gate.admit(command(), "watchdog-rollback", "lease", true, t + 700ms), "watchdog rollback setup failed");
    check(gate.stopPending(t + 699ms), "watchdog clock rollback did not latch stop");
    check(gate.stopPending(t + 701ms), "watchdog rollback stop lost before publish");
    gate.grant("bypass", "lease", t + 702ms, 1000ms);
    check(!gate.admit(command(), "bypass", "lease", true, t + 702ms), "watchdog rollback stop bypassed");
    gate.acknowledgeStopPublished();
    gate.grant("recovered", "lease", t + 703ms, 1000ms);
    check(gate.admit(command(), "recovered", "lease", true, t + 703ms), "post-stop recovery failed");
    gate.revoke();
    check(gate.stopPending(t + 704ms), "recovered motion did not require stop");
    gate.acknowledgeStopPublished();
    // Transport callbacks and the ROS wall-timer run on independent threads.
    // Concurrent revoke/grant/watchdog must not race on gate state.
    VelocityGate concurrent;
    const auto tick = t + 2s;
    std::thread transport([&] {
        for (int i = 0; i < 1000; ++i) {
            concurrent.grant("session", "lease", tick, 1000ms);
            concurrent.admit(command(static_cast<unsigned>(i + 1)), "session", "lease", true, tick);
            concurrent.revoke();
        }
    });
    std::thread watchdog([&] {
        for (int i = 0; i < 1000; ++i) {
            if (concurrent.stopPending(tick + 101ms))
                concurrent.acknowledgeStopPublished();
        }
    });
    transport.join();
    watchdog.join();
    concurrent.revoke();
    if (concurrent.stopPending(tick + 101ms))
        concurrent.acknowledgeStopPublished();
    check(!concurrent.stopPending(tick + 102ms), "concurrent stop did not drain");

}
