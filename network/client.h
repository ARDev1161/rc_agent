#ifndef GCLIENT_H
#define GCLIENT_H

#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "robot.grpc.pb.h"

using namespace Robot;

/**
 * @brief gRPC client for communication with the robot server.
 *
 * The grpcClient class encapsulates the functionality for sending sensor data and receiving control commands
 * from the robot server using gRPC. It provides methods for synchronous data exchange, bidirectional streaming,
 * and map streaming.
 */
class grpcClient
{
    /// Stub for making RPC calls.
    std::unique_ptr<ClientOnRobot::Stub> stub_;
    /// Shared pointer to the gRPC channel.
    std::shared_ptr<grpc::Channel> clientChannel;

    /// Shared pointer to the Controls object to be updated from the server.
    std::shared_ptr<Controls> controls_;
    /// Shared pointer to the Sensors object for sending sensor data.
    std::shared_ptr<Sensors> sensors_;
    /// Shared pointer to the GetMapResponse protobuf object for map data.
    std::shared_ptr<map_service::GetMapResponse> map_;

    /// Flag indicating whether the streaming has been stopped.
    // A stopped instance never restarts; reconnect constructs a new client.
    std::atomic<bool> stoppedStream{false};
    std::mutex muActiveContexts;
    std::vector<grpc::ClientContext*> activeContexts;
    bool registerStreamContext(grpc::ClientContext &context);
    void unregisterStreamContext(grpc::ClientContext &context);
    /// Flag to indicate whether to retry connection if it fails.
    bool retryConnect;
    /// Delay (in milliseconds) between connection retry attempts.
    int retryDelayMilliseconds = 1000;

    /// Mutex to protect client data.
    std::shared_ptr<std::mutex> muClient;
    /// Mutex to protect map data.
    std::shared_ptr<std::mutex> muMap;

 public:
    /**
     * @brief Constructs a new grpcClient object.
     *
     * Initializes the client stub, sets the channel and references to Controls, Sensors, and Map objects.
     *
     * @param channel Shared pointer to the gRPC channel.
     * @param controlsPtr Shared pointer to the Controls object.
     * @param sensorsPtr Shared pointer to the Sensors object.
     * @param mapPtr Shared pointer to the GetMapResponse protobuf object.
     * @param tryConnectIfFailed Flag indicating whether to retry connecting if the RPC fails.
     */
    grpcClient(std::shared_ptr<grpc::Channel> channel,
               std::shared_ptr<Controls> controlsPtr,
               std::shared_ptr<Sensors> sensorsPtr,
               std::shared_ptr<map_service::GetMapResponse> mapPtr,
               bool tryConnectIfFailed = true);

    /**
     * @brief Performs a synchronous data exchange with the server.
     *
     * Sends the sensor data to the server and receives control commands.
     *
     * @return grpc::Status Status of the RPC call.
     */
    grpc::Status DataExchange();

    /**
     * @brief Performs a bidirectional streaming data exchange.
     *
     * Continuously sends sensor data and receives control commands in a stream.
     *
     * @return grpc::Status Status of the streaming RPC call.
     */
    grpc::Status DataStreamExchange();

    /**
     * @brief Performs a streaming exchange for map data.
     *
     * Continuously sends map data and receives updates from the server.
     *
     * @return grpc::Status Status of the map streaming RPC call.
     */
    grpc::Status MapStream();

    /**
     * @brief Performs a streaming exchange for robot pose data.
     *
     * Continuously sends the latest robot pose and receives updates from the server.
     *
     * @return grpc::Status Status of the pose streaming RPC call.
     */
    grpc::Status PoseStream();

    /**
     * @brief Performs a streaming exchange for zone annotations.
     *
     * Continuously sends the latest zone map and receives updates from the server.
     *
     * @return grpc::Status Status of the zone streaming RPC call.
     */
    grpc::Status ZoneStream();

    /**
     * @brief Stops the current data stream.
     *
     * Cancels in-flight stream contexts (including blocked Read/Write).
     * This is transport cancellation, NOT a robot velocity safety watchdog.
     * A stopped instance cannot restart: reconnect creates a new client.
     */
    void stopStream();

    /**
     * @brief Returns a shared pointer to the client mutex.
     *
     * @return std::shared_ptr<std::mutex> Shared pointer to the mutex protecting client data.
     */
    std::shared_ptr<std::mutex> getMutex() { return muClient; }

    /**
     * @brief Returns a shared pointer to the map mutex.
     *
     * @return std::shared_ptr<std::mutex> Shared pointer to the mutex protecting map data.
     */
    std::shared_ptr<std::mutex> getMapMutex() { return muMap; }
};

#endif // GCLIENT_H
