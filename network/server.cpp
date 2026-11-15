#include "server.h"

void grpcServer::setProtosPointers(std::shared_ptr<Controls> controlsPtr, std::shared_ptr<Sensors> sensorsPtr)
{
    std::lock_guard<std::mutex> lock(muServer);
    controls_ = std::move(controlsPtr);
    sensors_ = std::move(sensorsPtr);
}

grpc::Status grpcServer::DataExchange([[maybe_unused]] grpc::ServerContext* context,
                                      const Controls* request, Sensors* reply)
{
    std::lock_guard<std::mutex> lock(muServer);
    if (!controls_ || !sensors_)
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state unavailable");
    *controls_ = *request;
    *reply = *sensors_;
    return grpc::Status::OK;
}

grpc::Status grpcServer::DataStreamExchange([[maybe_unused]] grpc::ServerContext* context,
                                            grpc::ServerReaderWriter<Sensors, Controls>* stream)
{
    // Never hold the shared control mutex across blocking gRPC Read/Write.
    // The ROS control loop must remain able to inspect and update state.
    Controls incoming;
    while (stream->Read(&incoming))
    {
        Sensors outgoing;
        {
            std::lock_guard<std::mutex> lock(muServer);
            if (!controls_ || !sensors_)
                return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state unavailable");
            *controls_ = incoming;
            outgoing = *sensors_;
        }
        if (!stream->Write(outgoing))
            break;
    }
    return grpc::Status::OK;
}
