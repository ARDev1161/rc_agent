#pragma once
// Transport-independent, fail-closed v2 velocity admission gate.
// The gRPC handler must authenticate the session and grant/revoke the lease;
// constructing this gate does not enable remote motion.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include "rc_api_v2.pb.h"

namespace rc::api::v2 {
class VelocityGate {
public:
    using Clock = std::chrono::steady_clock;
    void grant(const std::string& session, const std::string& lease, Clock::time_point now) {
        revoke();
        if (session.empty() || lease.empty() || pending_stop_) return;
        session_ = session;
        lease_ = lease;
        lease_started_ = now;
        active_ = true;
    }
    void revoke() {
        if (active_ && has_sequence_) pending_stop_ = true;
        active_ = false;
        session_.clear();
        lease_.clear();
        has_sequence_ = false;
        last_sequence_ = 0;
        deadline_ = Clock::time_point{};
        // Preserve an outstanding stop request when motion is revoked.
        // The ROS watchdog must drain this before accepting another lease.
    }
    bool admit(const Velocity& cmd, const std::string& session,
               const std::string& lease, bool deadman, Clock::time_point now) {
        if (pending_stop_ || !active_ || !deadman || session != session_ || lease != lease_ ||
            now < lease_started_ || (has_sequence_ && now >= deadline_) ||
            cmd.valid_for_ms() == 0 || cmd.valid_for_ms() > 1000 ||
            (has_sequence_ && cmd.sequence() <= last_sequence_) ||
            cmd.frame_id().empty()) {
            revoke(); // never retain a stale motion lease after an invalid command
            return false;
        }
        const auto finite = [](const Vector3& v) {
            return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        };
        if (!finite(cmd.linear()) || !finite(cmd.angular())) {
            revoke();
            return false;
        }
        deadline_ = now + std::chrono::milliseconds(cmd.valid_for_ms());
        last_sequence_ = cmd.sequence();
        has_sequence_ = true;
        return true;
    }
    bool expired(Clock::time_point now) const {
        return !active_ || !has_sequence_ || now >= deadline_;
    }
    // Called by the eventual ROS watchdog on a monotonic timer. Returns true
    // exactly once when an admitted motion command has timed out; the caller
    // MUST publish a zero Twist and revoke its authenticated transport lease.
    // No ROS publisher is owned by this transport-independent primitive.
    // Two-phase watchdog API: keep the stop latched until the ROS publisher
    // has actually accepted the zero-velocity publish operation. A failed
    // publish must never silently unlock a new lease.
    bool stopPending(Clock::time_point now) {
        if (active_ && has_sequence_ && now >= deadline_) revoke();
        return pending_stop_;
    }
    void acknowledgeStopPublished() {
        pending_stop_ = false;
    }
    bool consumeExpiredMotion(Clock::time_point now) {
        if (active_ && has_sequence_ && now >= deadline_) revoke();
        if (!pending_stop_) return false;
        pending_stop_ = false;
        return true;
    }
private:
    std::string session_, lease_;
    Clock::time_point lease_started_{}, deadline_{};
    std::uint64_t last_sequence_ = 0;
    bool active_ = false, has_sequence_ = false, pending_stop_ = false;
};
} // namespace rc::api::v2
