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
    stream_ctx.TryCancel();
    stream->WritesDone();
    stream->Finish();
    server->Shutdown();
}
