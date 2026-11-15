#include "network/v2_velocity_gate.h"
#include <chrono>
#include <stdexcept>
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
    gate.grant("s", "l", t);
    check(!gate.admit(command(), "wrong", "l", true, t), "wrong session accepted");
    gate.grant("s", "l", t);
    check(!gate.admit(command(), "s", "wrong", true, t), "wrong lease accepted");
    gate.grant("s", "l", t);
    check(!gate.admit(command(), "s", "l", false, t), "deadman bypass");
    gate.grant("s", "l", t);
    check(gate.admit(command(), "s", "l", true, t), "valid command rejected");
    check(!gate.stopPending(t + 99ms), "premature watchdog stop");
    check(gate.stopPending(t + 100ms), "watchdog stop missing");
    check(gate.stopPending(t + 101ms), "unacknowledged stop lost");
    gate.grant("next", "l2", t + 102ms);
    check(!gate.admit(command(), "next", "l2", true, t + 102ms), "pending stop bypassed");
    gate.acknowledgeStopPublished();
    gate.grant("next", "l2", t + 103ms);
    check(gate.admit(command(), "next", "l2", true, t + 103ms), "fresh lease rejected");
    check(!gate.admit(command(), "next", "l2", true, t + 104ms), "sequence replay accepted");
    check(gate.stopPending(t + 104ms), "replay did not revoke motion");
    gate.acknowledgeStopPublished();
    gate.grant("disconnect", "l3", t + 200ms);
    check(gate.admit(command(), "disconnect", "l3", true, t + 200ms), "setup failed");
    gate.revoke();
    check(gate.stopPending(t + 201ms), "disconnect stop missing");
    check(gate.stopPending(t + 500ms), "stop lost before DDS acknowledgement");
    check(!gate.admit(command(2), "disconnect", "l3", true, t + 201ms), "stale session accepted");
    gate.acknowledgeStopPublished();
    check(!gate.stopPending(t + 501ms), "acknowledged stop still pending");
}
