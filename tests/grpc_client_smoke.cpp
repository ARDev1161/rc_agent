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

    grpc::Status DataExchange(grpc::ServerContext *,
                              const Robot::Sensors *incoming,
                              Robot::Controls *reply) override {
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
        capture(incoming);
        received = true;
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
