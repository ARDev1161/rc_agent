#include "server.h"

namespace {
// The generic Controls RPC has no authenticated v2 session or lease.
// Never accept motion on this transport while the v2 velocity path is gated.
bool containsForbiddenMotion(const Controls& controls)
{
    if (controls.has_wheelmotors() &&
        (controls.wheelmotors().leftpower() != 0 || controls.wheelmotors().rightpower() != 0)) return true;
    if (controls.has_handmotors() &&
        (controls.handmotors().leftpower() != 0 || controls.handmotors().rightpower() != 0)) return true;
    if (!controls.has_basecontrol()) return false;
    const auto& velocity = controls.basecontrol();
    return velocity.linearvelocity() != 0.0 || velocity.angularvelocity() != 0.0;
}
} // namespace

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
    if (containsForbiddenMotion(*request))
        return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, "motion requires authorized v2 control path");
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
            if (containsForbiddenMotion(incoming))
                return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, "motion requires authorized v2 control path");
            *controls_ = incoming;
            outgoing = *sensors_;
        }
        if (!stream->Write(outgoing)) {
            // A cancelled transport must not report a successful stream close.
            // A graceful client half-close remains OK.
            if (context->IsCancelled())
                return grpc::Status(grpc::StatusCode::CANCELLED, "client disconnected");
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "sensor stream write failed");
        }
    }
    if (context->IsCancelled())
        return grpc::Status(grpc::StatusCode::CANCELLED, "client disconnected");
    // A graceful half-close is OK only for the same bound transport state.
    // Otherwise a reconnect racing with Read() == false could appear successful.
    {
        std::lock_guard<std::mutex> lock(muServer);
        if (!controls_ || !sensors_ || state_generation_ != bound_generation ||
            controls_ != bound_controls || sensors_ != bound_sensors)
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "transport state generation changed");
    }
    return grpc::Status::OK;
}
