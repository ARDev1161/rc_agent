// Real gRPC loopback regression for rc_agent's robot-side server.
// Exercises state snapshots and proves idle stream reads do not hold muServer.
#include "network/server.h"
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    grpcServer service;
    grpc::ServerBuilder builder;
    int port = 0;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&service);
    auto server = builder.BuildAndStart();
    require(server && port > 0, "server failed to start");
    auto channel = grpc::CreateChannel("127.0.0.1:" + std::to_string(port),
                                       grpc::InsecureChannelCredentials());
    require(channel->WaitForConnected(std::chrono::system_clock::now() +
                                      std::chrono::seconds(3)), "connection failed");
    auto stub = Robot::ServerOnRobot::NewStub(channel);
    Robot::Controls request;
    Robot::Sensors reply;
    {
        grpc::ClientContext ctx;
        auto status = stub->DataExchange(&ctx, request, &reply);
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "uninitialized unary exchange did not fail closed");
    }
    // An uninitialized streaming exchange must fail closed, just like unary.
    {
        grpc::ClientContext ctx;
        ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto uninitialized = stub->DataStreamExchange(&ctx);
        require(uninitialized->Write(request), "uninitialized stream write failed");
        Robot::Sensors ignored;
        require(!uninitialized->Read(&ignored), "uninitialized stream emitted sensors");
        auto status = uninitialized->Finish();
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "uninitialized streaming exchange did not fail closed");
    }
    // A client that never writes must not pin a gRPC worker when state is absent.
    // The server must reject the stream before its first blocking Read().
    {
        grpc::ClientContext ctx;
        ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto idle = stub->DataStreamExchange(&ctx);
        Robot::Sensors ignored;
        const auto started = std::chrono::steady_clock::now();
        require(!idle->Read(&ignored), "idle uninitialized stream unexpectedly emitted sensors");
        const auto status = idle->Finish();
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "idle uninitialized stream was not rejected immediately");
        require(std::chrono::steady_clock::now() - started < std::chrono::seconds(2),
                "uninitialized stream waited for a client message");
    }
    auto controls = std::make_shared<Robot::Controls>();
    auto sensors = std::make_shared<Robot::Sensors>();
    sensors->mutable_robot_info()->set_robot_id("robot-test");
    sensors->mutable_robot_info()->set_api_major(2);
    service.setProtosPointers(controls, sensors);
    {
        grpc::ClientContext ctx;
        auto status = stub->DataExchange(&ctx, request, &reply);
        require(status.ok(), "initialized unary exchange failed");
        require(reply.robot_info().robot_id() == "robot-test", "snapshot lost robot id");
    }

    grpc::ClientContext stream_ctx;
    stream_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(5));
    auto stream = stub->DataStreamExchange(&stream_ctx);
    require(stream->Write(request), "first stream write failed");
    Robot::Sensors streamed;
    require(stream->Read(&streamed), "first stream reply missing");
    require(streamed.robot_info().robot_id() == "robot-test", "stream snapshot mismatch");

    // Keep the stream idle at Read(). A concurrent unary RPC must still finish.
    auto concurrent = std::async(std::launch::async, [&] {
        grpc::ClientContext ctx;
        ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(2));
        Robot::Sensors output;
        return stub->DataExchange(&ctx, request, &output).ok() &&
               output.robot_info().robot_id() == "robot-test";
    });
    auto ready = concurrent.wait_for(std::chrono::milliseconds(1200));
    if (ready != std::future_status::ready) {
        stream_ctx.TryCancel();
        stream->WritesDone();
        stream->Finish();
        server->Shutdown();
        require(false, "idle stream blocked concurrent unary RPC");
    }
    require(concurrent.get(), "concurrent unary RPC failed");

    // Replacing the shared state while the stream is idle must not deadlock
    // and the next request must observe the new coherent sensor snapshot.
    auto replacement_controls = std::make_shared<Robot::Controls>();
    auto replacement_sensors = std::make_shared<Robot::Sensors>();
    replacement_sensors->mutable_robot_info()->set_robot_id("robot-reconnected");
    replacement_sensors->mutable_robot_info()->set_api_major(2);
    auto replace_state = std::async(std::launch::async, [&] {
        service.setProtosPointers(replacement_controls, replacement_sensors);
    });
    if (replace_state.wait_for(std::chrono::milliseconds(1200)) != std::future_status::ready) {
        stream_ctx.TryCancel();
        server->Shutdown();
        require(false, "idle stream blocked transport state replacement");
    }
    replace_state.get();
    // A stream belongs to the state generation present when it opened.
    // It must NOT transfer control authority to a replacement robot.
    (void)stream->Write(request);
    Robot::Sensors replacement_reply;
    require(!stream->Read(&replacement_reply),
            "stale stream emitted a reply after robot state replacement");
    const auto stale_status = stream->Finish();
    require(stale_status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
            "stale stream did not reject the changed robot state generation");
    require(replacement_controls->light().ledleftpower() == 0,
            "stale stream changed replacement robot controls");

    // A fresh connection may exchange telemetry and controls with the new state.
    {
        grpc::ClientContext fresh_ctx;
        fresh_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto fresh = stub->DataStreamExchange(&fresh_ctx);
        Robot::Controls second_command;
        second_command.mutable_light()->set_ledleftpower(73);
        require(fresh->Write(second_command), "fresh stream write failed");
        Robot::Sensors second_reply;
        require(fresh->Read(&second_reply), "fresh stream reply missing");
        require(replacement_controls->light().ledleftpower() == 73,
                "fresh stream did not update replacement control state");
        require(controls->light().ledleftpower() == 0,
                "detached robot controls changed after replacement");
        require(second_reply.robot_info().robot_id() == "robot-reconnected",
                "fresh stream returned stale robot descriptor");
        fresh_ctx.TryCancel();
        fresh->WritesDone();
        fresh->Finish();
    }
    // A cancelled RPC must not prevent a new stream from using current state.
    {
        grpc::ClientContext cancelled_ctx;
        cancelled_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto cancelled = stub->DataStreamExchange(&cancelled_ctx);
        Robot::Controls initial;
        initial.mutable_light()->set_ledleftpower(44);
        require(cancelled->Write(initial), "cancelled stream initial write failed");
        Robot::Sensors initial_reply;
        require(cancelled->Read(&initial_reply), "cancelled stream initial reply missing");
        cancelled_ctx.TryCancel();
        require(!cancelled->Finish().ok(), "cancelled stream unexpectedly completed normally");
        require(replacement_controls->light().ledleftpower() == 44,
                "cancelled stream corrupted last accepted control snapshot");
        grpc::ClientContext next_ctx;
        next_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto next = stub->DataStreamExchange(&next_ctx);
        Robot::Controls next_command;
        next_command.mutable_light()->set_ledleftpower(45);
        require(next->Write(next_command), "post-cancellation stream write failed");
        Robot::Sensors next_reply;
        require(next->Read(&next_reply), "post-cancellation stream reply missing");
        require(replacement_controls->light().ledleftpower() == 45,
                "post-cancellation stream did not update current control state");
        next->WritesDone();
        require(next->Finish().ok(), "post-cancellation stream failed to finish");
    }
    // Rebinding identical shared pointers is still a new transport generation.
    // A reconnect must revoke old streams even when ROS reused its state objects.
    {
        grpc::ClientContext rebound_ctx;
        rebound_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto rebound = stub->DataStreamExchange(&rebound_ctx);
        Robot::Controls first;
        first.mutable_light()->set_ledleftpower(73);
        require(rebound->Write(first), "same-pointer stream initial write failed");
        Robot::Sensors first_reply;
        require(rebound->Read(&first_reply), "same-pointer stream initial reply missing");
        service.setProtosPointers(replacement_controls, replacement_sensors);
        Robot::Controls stale_command;
        stale_command.mutable_light()->set_ledleftpower(91);
        (void)rebound->Write(stale_command);
        Robot::Sensors stale_reply;
        require(!rebound->Read(&stale_reply),
                "stream survived same-pointer transport generation change");
        const auto status = rebound->Finish();
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "same-pointer rebind did not revoke stale stream");
        require(replacement_controls->light().ledleftpower() == 73,
                "stale stream changed controls after same-pointer rebind");
    }
    // Removing the transport state must fail closed for both RPC shapes.
    // A newly opened idle stream must not wait for its first client write.
    service.setProtosPointers(nullptr, nullptr);
    {
        grpc::ClientContext ctx;
        Robot::Sensors unavailable_reply;
        const auto status = stub->DataExchange(&ctx, request, &unavailable_reply);
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "unary RPC accepted controls after transport state removal");
    }
    {
        grpc::ClientContext ctx;
        ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(3));
        auto idle_after_disconnect = stub->DataStreamExchange(&ctx);
        Robot::Sensors unavailable_reply;
        require(!idle_after_disconnect->Read(&unavailable_reply),
                "idle stream emitted sensors after transport state removal");
        const auto status = idle_after_disconnect->Finish();
        require(status.error_code() == grpc::StatusCode::FAILED_PRECONDITION,
                "idle stream after disconnect did not fail closed");
    }
    // The stale stream was already finalized above. gRPC permits exactly one
    // Finish() per client stream; a second Finish() aborts on older runtimes.
    server->Shutdown();
}
