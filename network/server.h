#ifndef GSERVER_H
#define GSERVER_H

#include <iostream>

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

#include "robot.grpc.pb.h"

using namespace Robot;

/**
 * @brief gRPC Server for robot control.
 *
 * The grpcServer class implements the ServerOnRobot::Service interface to handle
 * synchronous and streaming data exchanges between the server and clients.
 * It manages control and sensor data using internal mutex for thread safety.
 */
class grpcServer final : public ServerOnRobot::Service
{
    /// Shared pointer to the Controls object containing command data.
    std::shared_ptr<Controls> controls_;
    /// Shared pointer to the Sensors object containing sensor data.
    std::shared_ptr<Sensors> sensors_;

    /// Mutex for protecting server data during RPC calls.
    std::mutex muServer;

    /**
     * @brief Handles a synchronous data exchange RPC.
     *
     * Copies the received control data from the client into the server's Controls object,
     * then returns the current sensor data in the reply.
     *
     * @param context Server context.
     * @param request Pointer to the received Controls message.
     * @param reply Pointer to the Sensors message to be sent as response.
     * @return grpc::Status Status of the RPC call.
     */
    grpc::Status DataExchange([[maybe_unused]] grpc::ServerContext* context,
                              const Controls* request, Sensors* reply) override;

    /**
     * @brief Handles a bidirectional streaming RPC for data exchange.
     *
     * Continuously reads incoming control messages from the client and responds with the current sensor data.
     *
     * @param context Server context.
     * @param stream Pointer to the ServerReaderWriter for exchanging Sensors and Controls.
     * @return grpc::Status Status of the streaming RPC call.
     */
    grpc::Status DataStreamExchange([[maybe_unused]] grpc::ServerContext* context,
                                    grpc::ServerReaderWriter<Sensors, Controls>* stream) override;

public:
    /**
     * @brief Sets the shared pointers for control and sensor data.
     *
     * This function assigns the provided shared pointers to the internal variables used for handling RPCs.
     *
     * @param controlsPtr Shared pointer to the Controls object.
     * @param sensorsPtr Shared pointer to the Sensors object.
     */
    void setProtosPointers(std::shared_ptr<Controls> controlsPtr, std::shared_ptr<Sensors> sensorsPtr);

    /**
     * @brief Checks the connection status.
     *
     * @return int Status code indicating the connection state.
     */
    int checkConn();
};

#endif // GSERVER_H
