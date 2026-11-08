#include "client.h"
#include <cstdint>
#include <algorithm>
#include <grpcpp/grpcpp.h>

namespace {

template<typename UnregisterFn>
struct ActiveContextGuard {
    grpc::ClientContext &context;
    UnregisterFn &unregisterContext;
    ~ActiveContextGuard() { unregisterContext(context); }
};

template<typename StreamT, typename FactoryFn, typename PumpFn,
         typename RegisterFn, typename UnregisterFn>
grpc::Status runStreamWithRetry(const std::string &stream_name,
                                std::atomic<bool> &stopped_flag,
                                int retry_delay_ms,
                                bool retry_enabled,
                                RegisterFn &&register_context,
                                UnregisterFn &&unregister_context,
                                FactoryFn &&stream_factory,
                                PumpFn &&pump_loop)
{
    grpc::Status status(grpc::StatusCode::CANCELLED, "Stream was stopped");

    while (!stopped_flag.load()) {
        grpc::ClientContext context;
        if (!register_context(context)) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Stream was stopped");
        }
        ActiveContextGuard<UnregisterFn> context_guard{context, unregister_context};
        auto stream = stream_factory(context);
        if (!stream) {
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "Stream factory returned null");
        }

        pump_loop(*stream);

        stream->WritesDone();
        status = stream->Finish();
        if (status.ok()) {
            return status;
        } else {
            std::cout << "Error " << status.error_code() << " : " << status.error_message() << std::endl;
            std::cout << stream_name << " rpc failed." << std::endl;
            if (!retry_enabled || stopped_flag.load()) {
                break;
            }
            constexpr int kSleepStepMs = 10;
            for (int elapsed = 0;
                 elapsed < retry_delay_ms && !stopped_flag.load();
                 elapsed += kSleepStepMs) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(std::min(kSleepStepMs, retry_delay_ms - elapsed)));
            }
        }
    }

    return status;
}

bool poseEquals(const map_service::Pose &a, const map_service::Pose &b)
{
    return a.position_x() == b.position_x() &&
           a.position_y() == b.position_y() &&
           a.position_z() == b.position_z() &&
           a.orientation_x() == b.orientation_x() &&
           a.orientation_y() == b.orientation_y() &&
           a.orientation_z() == b.orientation_z() &&
           a.orientation_w() == b.orientation_w();
}

} // namespace

grpcClient::grpcClient(std::shared_ptr<grpc::Channel> channel,
               std::shared_ptr<Controls> controlsPtr,
               std::shared_ptr<Sensors> sensorsPtr,
               std::shared_ptr<map_service::GetMapResponse> mapPtr,
               bool tryConnectIfFailed)
      : stub_(ClientOnRobot::NewStub(channel)),
        retryConnect(tryConnectIfFailed),
        controls_(controlsPtr),
        sensors_(sensorsPtr),
        map_(mapPtr),
        muClient(std::make_shared<std::mutex>()),
        muMap(std::make_shared<std::mutex>())
{
    clientChannel = channel;
    std::cout << "Client started" << std::endl;
}

// Assembles the client's payload, sends it and presents the response back from the server.
grpc::Status grpcClient::DataExchange()
{
    for (;;) {
        if (stoppedStream.load()) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client was stopped");
        }
        Sensors snapshot;
        {
            std::lock_guard<std::mutex> lock(*muClient);
            snapshot = *sensors_;
        }
        // Bound each attempt and never hold ROS callback mutex across I/O.
        Controls reply;
        grpc::ClientContext context;
        if (!registerStreamContext(context)) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client was stopped");
        }
        // Registration also lets stopStream interrupt an in-flight unary RPC.
        auto unregister = [this](grpc::ClientContext &ctx) { unregisterStreamContext(ctx); };
        ActiveContextGuard<decltype(unregister)> context_guard{context, unregister};
        context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        grpc::Status status = stub_->DataExchange(&context, snapshot, &reply);
        if (stoppedStream.load()) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client was stopped");
        }
        if (status.ok()) {
            std::lock_guard<std::mutex> lock(*muClient);
            if (stoppedStream.load()) {
                return grpc::Status(grpc::StatusCode::CANCELLED, "Client was stopped");
            }
            *controls_ = std::move(reply);
            return status;
        }
        std::cout << "DataExchange rpc failed: " << status.error_message() << std::endl;
        if (!retryConnect) {
            return status;
        }
        constexpr int kSleepStepMs = 10;
        for (int elapsed = 0;
             elapsed < retryDelayMilliseconds && !stoppedStream.load();
             elapsed += kSleepStepMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                std::min(kSleepStepMs, retryDelayMilliseconds - elapsed)));
        }
    }
}

bool grpcClient::registerStreamContext(grpc::ClientContext &context)
{
    std::lock_guard<std::mutex> lock(muActiveContexts);
    if (stoppedStream.load()) {
        return false;
    }
    activeContexts.push_back(&context);
    return true;
}

void grpcClient::unregisterStreamContext(grpc::ClientContext &context)
{
    std::lock_guard<std::mutex> lock(muActiveContexts);
    const auto it = std::find(activeContexts.begin(), activeContexts.end(), &context);
    if (it != activeContexts.end()) {
        activeContexts.erase(it);
    }
}

void grpcClient::stopStream()
{
    stoppedStream.store(true);
    std::lock_guard<std::mutex> lock(muActiveContexts);
    for (auto *context : activeContexts) {
        context->TryCancel();
    }
    std::cout << "Client stopped" << std::endl;
}

// Rely on stream Write/Read status, not GetState(): new channels can be CONNECTING.
// Gating on READY before the first write silently loses the initial snapshot.
grpc::Status grpcClient::DataStreamExchange()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<Sensors, Controls>>(
        "DataStreamExchange",
        stoppedStream,
        retryDelayMilliseconds,
        retryConnect,
        [this](grpc::ClientContext &context) { return registerStreamContext(context); },
        [this](grpc::ClientContext &context) { unregisterStreamContext(context); },
        [this](grpc::ClientContext &context) {
            return std::shared_ptr<grpc::ClientReaderWriter<Sensors, Controls>>(
                stub_->DataStreamExchange(&context));
        },
        [this](grpc::ClientReaderWriter<Sensors, Controls> &stream) {
            while (!stoppedStream)
            {
                Sensors outgoing;
                {
                    // ROS callbacks must not wait on potentially stalled network I/O.
                    std::lock_guard<std::mutex> lock(*muClient);
                    outgoing = *sensors_;
                }
                if (!stream.Write(outgoing)) {
                    break;
                }
                Controls incoming;
                if (!stream.Read(&incoming)) {
                    break;
                }
                {
                    std::lock_guard<std::mutex> lock(*muClient);
                    *controls_ = std::move(incoming);
                }
            }
        }
    );
}

grpc::Status grpcClient::MapStream()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(
        "MapStream",
        stoppedStream,
        retryDelayMilliseconds,
        retryConnect,
        [this](grpc::ClientContext &context) { return registerStreamContext(context); },
        [this](grpc::ClientContext &context) { unregisterStreamContext(context); },
        [this](grpc::ClientContext &context) {
            // Never hold the ROS map mutex while creating an RPC stream.
            {
                std::lock_guard<std::mutex> lock(*muMap);
                if (!map_) {
                    return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(nullptr);
                }
            }
            return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(
                stub_->MapStream(&context));
        },
        [this](grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest> &stream) {
            map_service::GetMapRequest request;
            std::uint64_t last_seq = 0;
            bool has_seq = false;
            const int idle_sleep_ms = 10;

            while (!stoppedStream)
            {
                map_service::GetMapResponse outgoing;
                bool changed = false;
                {
                    // Sequence and payload must describe the SAME snapshot.
                    // No map hashing, and no mutex held over gRPC Write/Read.
                    std::lock_guard<std::mutex> lock(*muMap);
                    if (map_ && (!has_seq || map_->map_seq() != last_seq)) {
                        outgoing = *map_;
                        last_seq = outgoing.map_seq();
                        has_seq = true;
                        changed = true;
                    }
                }
                if (!changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
                    continue;
                }
                if (!stream.Write(outgoing) || !stream.Read(&request)) {
                    break;  // Return to retry policy instead of spinning.
                }
            }
        }
    );
}

grpc::Status grpcClient::PoseStream()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<map_service::PoseState, map_service::PoseRequest>>(
        "PoseStream",
        stoppedStream,
        retryDelayMilliseconds,
        retryConnect,
        [this](grpc::ClientContext &context) { return registerStreamContext(context); },
        [this](grpc::ClientContext &context) { unregisterStreamContext(context); },
        [this](grpc::ClientContext &context) {
            {
                std::lock_guard<std::mutex> lock(*muMap);
                if (!map_) {
                    return std::shared_ptr<grpc::ClientReaderWriter<map_service::PoseState, map_service::PoseRequest>>(nullptr);
                }
            }
            return std::shared_ptr<grpc::ClientReaderWriter<map_service::PoseState, map_service::PoseRequest>>(
                stub_->PoseStream(&context));
        },
        [this](grpc::ClientReaderWriter<map_service::PoseState, map_service::PoseRequest> &stream) {
            map_service::PoseRequest request;
            map_service::PoseState state;
            map_service::Pose last_pose;
            bool has_pose = false;
            const int idle_sleep_ms = 10;

            while (!stoppedStream)
            {
                bool changed = false;
                {
                    std::unique_lock<std::mutex> lock(*muMap);
                    if (map_) {
                        *state.mutable_pose() = map_->robotpose();
                        if (!has_pose || !poseEquals(state.pose(), last_pose)) {
                            last_pose = state.pose();
                            has_pose = true;
                            changed = true;
                        }
                    }
                }
                if (!changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
                    continue;
                }
                if (!stream.Write(state) || !stream.Read(&request)) {
                    break;
                }
            }
        }
    );
}

grpc::Status grpcClient::ZoneStream()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<map_service::ZoneState, map_service::ZoneRequest>>(
        "ZoneStream",
        stoppedStream,
        retryDelayMilliseconds,
        retryConnect,
        [this](grpc::ClientContext &context) { return registerStreamContext(context); },
        [this](grpc::ClientContext &context) { unregisterStreamContext(context); },
        [this](grpc::ClientContext &context) {
            {
                std::lock_guard<std::mutex> lock(*muMap);
                if (!map_) {
                    return std::shared_ptr<grpc::ClientReaderWriter<map_service::ZoneState, map_service::ZoneRequest>>(nullptr);
                }
            }
            return std::shared_ptr<grpc::ClientReaderWriter<map_service::ZoneState, map_service::ZoneRequest>>(
                stub_->ZoneStream(&context));
        },
        [this](grpc::ClientReaderWriter<map_service::ZoneState, map_service::ZoneRequest> &stream) {
            map_service::ZoneRequest request;
            map_service::ZoneState state;
            std::uint64_t last_seq = 0;
            bool has_seq = false;
            const int idle_sleep_ms = 10;

            while (!stoppedStream)
            {
                bool changed = false;
                {
                    // Bind the zone payload to the very sequence we observed.
                    // Never perform RPC I/O with the map mutex held.
                    std::lock_guard<std::mutex> lock(*muMap);
                    if (map_ && (!has_seq || map_->zone_seq() != last_seq)) {
                        *state.mutable_zone_map() = map_->zone_map();
                        last_seq = map_->zone_seq();
                        has_seq = true;
                        changed = true;
                    }
                }
                if (!changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
                    continue;
                }
                if (!stream.Write(state) || !stream.Read(&request)) {
                    break;
                }
            }
        }
    );
}
