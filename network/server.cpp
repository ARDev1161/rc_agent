#include "server.h"

void grpcServer::setProtosPointers(std::shared_ptr<Controls> controlsPtr, std::shared_ptr<Sensors> sensorsPtr)
{
    std::lock_guard<std::mutex> lock(muServer);
    controls_ = std::move(controlsPtr);
    sensors_ = std::move(sensorsPtr);
    ++state_generation_;
}

grpc::Status grpcServer::DataExchange(grpc::ServerContext* context,
                                      const Controls* request, Sensors* reply)
{
    std::lock_guard<std::mutex> lock(muServer);
    if (context->IsCancelled())
        return grpc::Status(grpc::StatusCode::CANCELLED, "client disconnected");
    if (!controls_ || !sensors_)
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state unavailable");
    *controls_ = *request;
    *reply = *sensors_;
    return grpc::Status::OK;
}

grpc::Status grpcServer::DataStreamExchange(grpc::ServerContext* context,
                                            grpc::ServerReaderWriter<Sensors, Controls>* stream)
{
    // Never hold the shared control mutex across blocking gRPC Read/Write.
    // The ROS control loop must remain able to inspect and update state.
    std::shared_ptr<Controls> bound_controls;
    std::shared_ptr<Sensors> bound_sensors;
    std::uint64_t bound_generation = 0;
    {
        std::lock_guard<std::mutex> lock(muServer);
        // Validate and capture one atomic state generation before blocking Read.
        // A separate preflight lock would permit a null state to slip between
        // validation and binding, leaving an idle stream waiting indefinitely.
        if (!controls_ || !sensors_)
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state unavailable");
        bound_controls = controls_;
        bound_sensors = sensors_;
        bound_generation = state_generation_;
    }
    Controls incoming;
    while (stream->Read(&incoming))
    {
        Sensors outgoing;
        {
            std::lock_guard<std::mutex> lock(muServer);
            if (context->IsCancelled())
                return grpc::Status(grpc::StatusCode::CANCELLED, "client disconnected");
            if (!controls_ || !sensors_ || state_generation_ != bound_generation ||
                controls_ != bound_controls || sensors_ != bound_sensors)
                return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state generation changed");
            *controls_ = incoming;
            outgoing = *sensors_;
        }
        if (!stream->Write(outgoing))
            break;
    }
    return grpc::Status::OK;
}
