#include "client.h"
#include <cstdint>
#include <grpcpp/grpcpp.h>

namespace {

template<typename StreamT, typename FactoryFn, typename PumpFn>
grpc::Status runStreamWithRetry(const std::string &stream_name,
                                bool &stopped_flag,
                                int retry_delay_ms,
                                FactoryFn &&stream_factory,
                                PumpFn &&pump_loop)
{
    grpc::Status status;
    bool retry = true;
    stopped_flag = false;

    while (retry) {
        grpc::ClientContext context;
        auto stream = stream_factory(context);
        if (!stream) {
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "Stream factory returned null");
        }

        pump_loop(*stream);

        stream->WritesDone();
        status = stream->Finish();
        if (status.ok()) {
            retry = false;
        } else {
            std::cout << "Error " << status.error_code() << " : " << status.error_message() << std::endl;
            std::cout << stream_name << " rpc failed." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
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
    grpc::Status status;
    bool retry = true; // Flag that defined need connection retry or no

    while(retry){
        // Container for the data we expect from the server.
        Controls reply;

        // Context for the client. It could be used to convey extra information to
        // the server and/or tweak certain RPC behaviors.
        grpc::ClientContext context;

        // The actual RPC.
        status = stub_->DataExchange(&context, *sensors_, &reply);

        std::unique_lock<std::mutex> ul(muClient);

        // Act upon its status.
        if (status.ok()){
          *controls_ = reply;
          retry = false;
        }
        else{
          std::cout << "DataExchange rpc failed!" << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds( retryDelayMilliseconds ));
        }

    }

    return status;
}

void grpcClient::stopStream()
{
    stoppedStream = true;
    std::cout << "Client stopped" << std::endl;
}

grpc::Status grpcClient::DataStreamExchange()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<Sensors, Controls>>(
        "DataStreamExchange",
        stoppedStream,
        retryDelayMilliseconds,
        [this](grpc::ClientContext &context) {
            return std::shared_ptr<grpc::ClientReaderWriter<Sensors, Controls>>(
                stub_->DataStreamExchange(&context));
        },
        [this](grpc::ClientReaderWriter<Sensors, Controls> &stream) {
            while(!stoppedStream && (clientChannel->GetState(true) == GRPC_CHANNEL_READY))
            {
                std::unique_lock<std::mutex> lock(muClient);

                stream.Write(*sensors_);
                stream.Read(controls_.get());
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
        [this](grpc::ClientContext &context) {
            std::unique_lock<std::mutex> lock(muMap);
            if (!map_) {
                return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(nullptr);
            }
            return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(
                stub_->MapStream(&context));
        },
        [this](grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest> &stream) {
            map_service::GetMapRequest request;
            std::uint64_t last_seq = 0;
            bool has_seq = false;
            const int idle_sleep_ms = 10;

            while(!stoppedStream && (clientChannel->GetState(true) == GRPC_CHANNEL_READY))
            {
                bool changed = false;
                {
                    std::unique_lock<std::mutex> lock(muMap);
                    if (map_) {
                        std::uint64_t seq = map_->map_seq();
                        if (!has_seq || seq != last_seq) {
                            last_seq = seq;
                            has_seq = true;
                            changed = true;
                        }
                    }
                }
                if (!changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
                    continue;
                }
                {
                    std::unique_lock<std::mutex> lock(muMap);
                    if (map_) {
                        stream.Write(*map_);
                    }
                }
                stream.Read(&request);
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
        [this](grpc::ClientContext &context) {
            std::unique_lock<std::mutex> lock(muMap);
            if (!map_) {
                return std::shared_ptr<grpc::ClientReaderWriter<map_service::PoseState, map_service::PoseRequest>>(nullptr);
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

            while(!stoppedStream && (clientChannel->GetState(true) == GRPC_CHANNEL_READY))
            {
                bool changed = false;
                {
                    std::unique_lock<std::mutex> lock(muMap);
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
                stream.Write(state);
                stream.Read(&request);
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
        [this](grpc::ClientContext &context) {
            std::unique_lock<std::mutex> lock(muMap);
            if (!map_) {
                return std::shared_ptr<grpc::ClientReaderWriter<map_service::ZoneState, map_service::ZoneRequest>>(nullptr);
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

            while(!stoppedStream && (clientChannel->GetState(true) == GRPC_CHANNEL_READY))
            {
                bool changed = false;
                {
                    std::unique_lock<std::mutex> lock(muMap);
                    if (map_) {
                        std::uint64_t seq = map_->zone_seq();
                        if (!has_seq || seq != last_seq) {
                            last_seq = seq;
                            has_seq = true;
                            changed = true;
                        }
                    }
                }
                if (!changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
                    continue;
                }
                {
                    std::unique_lock<std::mutex> lock(muMap);
                    if (map_) {
                        *state.mutable_zone_map() = map_->zone_map();
                    }
                }
                stream.Write(state);
                stream.Read(&request);
            }
        }
    );
}
