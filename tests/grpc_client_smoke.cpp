// Headless integration test: the real rc_agent gRPC client against a loopback
// AmurCore-compatible service. No ROS2 executor or physical robot needed.
#include "network/client.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
void require(bool good, const char *message) {
    if (!good) throw std::runtime_error(message);
}
class OperatorMock final : public Robot::ClientOnRobot::Service {
public:
    std::atomic<bool> received{false};
    std::atomic<unsigned> stream_count{0};
    std::atomic<bool> fail_next_unary{false};
    std::atomic<bool> fail_next_stream{false};
    std::atomic<unsigned> unary_count{0};
    std::atomic<bool> hold_unary{false};
    std::atomic<bool> unary_waiting{false};
    std::atomic<bool> release_reply{false};
    std::mutex observed_mutex;
    std::string observed_robot_id;
    std::string observed_capability;
    std::string observed_axis;

    void capture(const Robot::Sensors &incoming) {
        std::lock_guard<std::mutex> lock(observed_mutex);
        observed_robot_id = incoming.robot_info().robot_id();
        observed_capability.clear();
        observed_axis.clear();
        if (incoming.robot_info().capabilities_size() > 0) {
            const auto &cap = incoming.robot_info().capabilities(0);
            observed_capability = cap.id();
            if (cap.velocity_axes_size() > 0) {
                observed_axis = cap.velocity_axes(0);
            }
        }
    }
    bool matchesDescriptor() {
        std::lock_guard<std::mutex> lock(observed_mutex);
        return observed_robot_id == "robot-test" &&
               observed_capability == "manual.velocity" &&
               observed_axis == "linear.x";
    }

    grpc::Status DataExchange(grpc::ServerContext *ctx,
                               const Robot::Sensors *incoming,
                               Robot::Controls *reply) override {
        ++unary_count;
        if (hold_unary.load()) {
            unary_waiting.store(true);
            while (hold_unary.load() && !ctx->IsCancelled()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (ctx->IsCancelled()) {
                return grpc::Status(grpc::StatusCode::CANCELLED, "Unary canceled");
            }
        }
        if (fail_next_unary.exchange(false)) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Simulated disconnect");
        }
        capture(*incoming);
        reply->mutable_basecontrol()->set_linearvelocity(11.0);
        return grpc::Status::OK;
    }

    grpc::Status DataStreamExchange(
        grpc::ServerContext *ctx,
        grpc::ServerReaderWriter<Robot::Controls, Robot::Sensors> *stream) override {
        Robot::Sensors incoming;
        if (!stream->Read(&incoming)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing telemetry");
        }
        if (fail_next_stream.exchange(false)) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Simulated stream disconnect");
        }
        capture(incoming);
        received = true;
        ++stream_count;
        while (!release_reply.load() && !ctx->IsCancelled()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ctx->IsCancelled()) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client canceled");
        }
        Robot::Controls reply;
        reply.mutable_basecontrol()->set_linearvelocity(11.0);
        stream->Write(reply);
        return grpc::Status::OK;
    }
};
}  // namespace

int main() {
    OperatorMock mock;
    grpc::ServerBuilder builder;
    int port = 0;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&mock);
    auto server = builder.BuildAndStart();
    require(server != nullptr && port != 0, "Could not start operator mock");

    const auto address = "127.0.0.1:" + std::to_string(port);
    auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
    require(channel->WaitForConnected(std::chrono::system_clock::now() +
                                      std::chrono::seconds(3)),
            "Could not connect to loopback service");

    auto controls = std::make_shared<Robot::Controls>();
    auto sensors = std::make_shared<Robot::Sensors>();
    auto map = std::make_shared<map_service::GetMapResponse>();
    sensors->mutable_robot_info()->set_robot_id("robot-test");
    sensors->mutable_robot_info()->set_api_major(2);
    auto *cap = sensors->mutable_robot_info()->add_capabilities();
    cap->set_id("manual.velocity");
    cap->add_velocity_axes("linear.x");
    grpcClient client(channel, controls, sensors, map, false);

    try {
        require(client.DataExchange().ok(), "Legacy-compatible unary RPC failed");
        require(mock.matchesDescriptor(), "RobotInfo was not delivered");
        {
            auto mutex = client.getMutex();
            std::lock_guard<std::mutex> lock(*mutex);
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Unary controls reply was not applied");
        }

        // Use a *fresh*, potentially CONNECTING channel to catch the startup
        // race where GetState() gating previously skipped the first snapshot.
        auto stream_channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
        grpcClient streaming_client(stream_channel, controls, sensors, map, false);
        auto rpc = std::async(std::launch::async, [&streaming_client]() {
            return streaming_client.DataStreamExchange();
        });
        for (int i = 0; i < 300 && !mock.received.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.received.load(), "Agent did not transmit first streaming telemetry");

        // The server holds its response for this test. ROS callbacks must still
        // be able to acquire the agent's shared protobuf mutex during the wait.
        auto mutex = streaming_client.getMutex();
        bool available = false;
        for (int i = 0; i < 40; ++i) {
            if (mutex->try_lock()) {
                mutex->unlock();
                available = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        mock.release_reply = true;
        require(rpc.wait_for(std::chrono::seconds(3)) == std::future_status::ready,
                "Client stream did not finish after server sent reply");
        require(rpc.get().ok(), "Client stream RPC failed");
        require(available, "ROS mutex blocked by gRPC response wait");
        require(mock.matchesDescriptor(), "Stream lost robot ID or capability/axes");
        {
            std::lock_guard<std::mutex> lock(*mutex);
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Stream controls were not applied");
        }

        // A legacy snapshot must clear metadata observed on the previous
        // v2 unary/stream. The mock must never retain stale capabilities.
        {
            auto client_mutex = client.getMutex();
            std::lock_guard<std::mutex> lock(*client_mutex);
            sensors->clear_robot_info();
        }
        require(client.DataExchange().ok(), "Legacy unary reconnect failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(mock.observed_robot_id.empty() &&
                        mock.observed_capability.empty() &&
                        mock.observed_axis.empty(),
                    "Legacy reconnect retained stale RobotInfo fields");
        }
        // False means one attempt, error propagation and no stale controls.
        mock.fail_next_unary = true;
        unsigned before = mock.unary_count.load();
        auto start = std::chrono::steady_clock::now();
        const grpc::Status failed = client.DataExchange();
        require(failed.error_code() == grpc::StatusCode::UNAVAILABLE,
                "One-shot unary did not propagate disconnect");
        require(mock.unary_count.load() == before + 1,
                "One-shot unary retried unexpectedly");
        require(std::chrono::steady_clock::now() - start < std::chrono::seconds(3),
                "One-shot unary did not fail promptly");
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Failed unary overwrote a valid control reply");
        }

        // One transient failure must recover when retry is enabled,
        // without blocking ROS callback access during the backoff.
        grpcClient retrying(channel, controls, sensors, map, true);
        mock.fail_next_unary = true;
        before = mock.unary_count.load();
        auto pending = std::async(std::launch::async, [&retrying]() {
            return retrying.DataExchange();
        });
        for (int i = 0; i < 100 && mock.unary_count.load() == before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.unary_count.load() == before + 1,
                "Retrying client did not start");
        bool unlocked = false;
        for (int i = 0; i < 40; ++i) {
            if (retrying.getMutex()->try_lock()) {
                retrying.getMutex()->unlock();
                unlocked = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(unlocked, "Retry backoff held ROS mutex");
        require(pending.wait_for(std::chrono::seconds(5)) == std::future_status::ready,
                "Retry after transient disconnect timed out");
        require(pending.get().ok(), "Retry-enabled unary did not recover");
        require(mock.unary_count.load() == before + 2,
                "Retry-enabled unary performed unexpected extra attempts");

        // A blocked unary RPC must be canceled by stopStream as well; no
        // stale controls may be applied, and a stopped client is terminal.
        mock.hold_unary.store(true);
        mock.unary_waiting.store(false);
        grpcClient unary_cancel(channel, controls, sensors, map, true);
        auto unary_pending = std::async(std::launch::async, [&unary_cancel]() {
            return unary_cancel.DataExchange();
        });
        for (int i = 0; i < 300 && !mock.unary_waiting.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool unary_started = mock.unary_waiting.load();
        unary_cancel.stopStream();
        const bool unary_finished =
            unary_pending.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        mock.hold_unary.store(false);
        require(unary_started, "Unary cancellation did not reach mock");
        require(unary_finished, "stopStream did not cancel unary RPC");
        require(unary_pending.get().error_code() == grpc::StatusCode::CANCELLED,
                "Stopped unary RPC returned unexpected status");
        require(unary_cancel.DataExchange().error_code() == grpc::StatusCode::CANCELLED,
                "Stopped client restarted unary RPC");
        {
            std::lock_guard<std::mutex> lock(*unary_cancel.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Canceled unary RPC changed previously valid controls");
        }

        // Simulate a server that holds the reply. stopStream must cancel
        // the outstanding gRPC Read and preserve the last controls snapshot.
        mock.received.store(false);
        mock.release_reply.store(false);
        grpcClient cancellable(channel, controls, sensors, map, true);
        auto blocked = std::async(std::launch::async, [&cancellable]() {
            return cancellable.DataStreamExchange();
        });
        for (int i = 0; i < 300 && !mock.received.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.received.load(), "Cancel test did not reach the mock");
        require(blocked.wait_for(std::chrono::milliseconds(50)) ==
                    std::future_status::timeout,
                "Mock did not block the stream response");
        const auto stop_start = std::chrono::steady_clock::now();
        cancellable.stopStream();
        require(blocked.wait_for(std::chrono::seconds(3)) ==
                    std::future_status::ready, "stopStream did not unblock Read");
        const grpc::Status stopped = blocked.get();
        require(stopped.error_code() == grpc::StatusCode::CANCELLED,
                "stopStream did not return CANCELLED");
        require(std::chrono::steady_clock::now() - stop_start <
                    std::chrono::seconds(3), "stopStream took too long");
        {
            std::lock_guard<std::mutex> lock(*cancellable.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Cancellation overwrote previously valid controls");
        }
        require(cancellable.DataStreamExchange().error_code() ==
                    grpc::StatusCode::CANCELLED,
                "Stopped client restarted a stream");

        // stopStream must cancel every in-flight context, not only the most
        // recently registered stream. Two calls share one client instance.
        mock.release_reply.store(false);
        const unsigned streams_before = mock.stream_count.load();
        grpcClient multi_stream(channel, controls, sensors, map, true);
        auto first = std::async(std::launch::async, [&multi_stream]() {
            return multi_stream.DataStreamExchange();
        });
        auto second = std::async(std::launch::async, [&multi_stream]() {
            return multi_stream.DataStreamExchange();
        });
        for (int i = 0; i < 300 &&
             mock.stream_count.load() < streams_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool both_started = mock.stream_count.load() == streams_before + 2;
        multi_stream.stopStream();
        multi_stream.stopStream(); // repeated shutdown is safe
        const bool both_finished =
            first.wait_for(std::chrono::seconds(3)) == std::future_status::ready &&
            second.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        require(both_started, "Concurrent streams did not reach operator");
        require(both_finished, "stopStream did not cancel both active contexts");
        require(first.get().error_code() == grpc::StatusCode::CANCELLED &&
                    second.get().error_code() == grpc::StatusCode::CANCELLED,
                "Concurrent stream cancellation returned wrong status");
        {
            std::lock_guard<std::mutex> lock(*multi_stream.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Concurrent cancellation changed valid controls");
        }

        // A single client may have a blocked unary and a blocked stream at
        // the same time. Shutdown must cancel both contexts, preserve the
        // last valid controls, and never retry either RPC afterwards.
        mock.hold_unary.store(true);
        mock.unary_waiting.store(false);
        mock.release_reply.store(false);
        const unsigned mixed_stream_before = mock.stream_count.load();
        grpcClient mixed(channel, controls, sensors, map, true);
        auto mixed_unary = std::async(std::launch::async, [&mixed]() {
            return mixed.DataExchange();
        });
        auto mixed_stream = std::async(std::launch::async, [&mixed]() {
            return mixed.DataStreamExchange();
        });
        for (int i = 0; i < 300 &&
             (!mock.unary_waiting.load() ||
              mock.stream_count.load() < mixed_stream_before + 1); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool mixed_both_started = mock.unary_waiting.load() &&
            mock.stream_count.load() >= mixed_stream_before + 1;
        mixed.stopStream();
        const bool mixed_both_finished =
            mixed_unary.wait_for(std::chrono::seconds(3)) == std::future_status::ready &&
            mixed_stream.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        mock.hold_unary.store(false);
        require(mixed_both_started, "Mixed unary and stream did not reach operator");
        require(mixed_both_finished, "Mixed unary and stream were not both canceled");
        require(mixed_unary.get().error_code() == grpc::StatusCode::CANCELLED &&
                    mixed_stream.get().error_code() == grpc::StatusCode::CANCELLED,
                "Mixed cancellation returned unexpected status");
        require(mixed.DataExchange().error_code() == grpc::StatusCode::CANCELLED &&
                    mixed.DataStreamExchange().error_code() == grpc::StatusCode::CANCELLED,
                "Mixed stopped client restarted an RPC");
        {
            std::lock_guard<std::mutex> lock(*mixed.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Mixed cancellation overwrote valid controls");
        }

        // The streaming transport must obey the same one-shot policy.
        mock.fail_next_stream = true;
        grpcClient one_shot_stream(channel, controls, sensors, map, false);
        start = std::chrono::steady_clock::now();
        const grpc::Status stream_failed = one_shot_stream.DataStreamExchange();
        require(stream_failed.error_code() == grpc::StatusCode::UNAVAILABLE,
                "One-shot stream did not surface peer failure");
        require(std::chrono::steady_clock::now() - start < std::chrono::seconds(3),
                "One-shot stream retried or blocked unexpectedly");
        {
            std::lock_guard<std::mutex> lock(*one_shot_stream.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Failed stream overwrote previously valid controls");
        }
    } catch (const std::exception &e) {
        mock.release_reply = true;
        server->Shutdown();
        std::cerr << "Agent RPC smoke failed: " << e.what() << std::endl;
        return 1;
    }
    server->Shutdown();
    std::cout << "rc_agent unary/stream and idle mutex smoke: PASS" << std::endl;
    return 0;
}
