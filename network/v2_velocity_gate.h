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
        if (session.empty() || lease.empty()) return;
        session_ = session;
        lease_ = lease;
        lease_started_ = now;
        active_ = true;
    }
    void revoke() {
        active_ = false;
        session_.clear();
        lease_.clear();
        has_sequence_ = false;
        last_sequence_ = 0;
        deadline_ = Clock::time_point{};
    }
    bool admit(const Velocity& cmd, const std::string& session,
               const std::string& lease, bool deadman, Clock::time_point now) {
        if (!active_ || !deadman || session != session_ || lease != lease_ ||
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
private:
    std::string session_, lease_;
    Clock::time_point lease_started_{}, deadline_{};
    std::uint64_t last_sequence_ = 0;
    bool active_ = false, has_sequence_ = false;
};
} // namespace rc::api::v2
