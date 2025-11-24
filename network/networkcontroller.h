#ifndef NETWORKCONTROLLER_H
#define NETWORKCONTROLLER_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "client.h"
#include "server.h"
#include "arper.h"

/**
 * @brief Controller class for managing network connections, client and server operations.
 *
 * The NetworkController class manages network operations by starting ARP services,
 * running gRPC clients and servers, and maintaining connection status and control machine addresses.
 */
class NetworkController : public IControlMachineObserver
{
public:
    /**
     * @brief Connection states for the network controller.
     */
    enum class ConnectionState {
        Discovering,
        Connecting,
        Connected,
        Reconnecting
    };

    /**
     * @brief Strategy interface for reconnection delays.
     */
    class ReconnectStrategy {
    public:
        virtual ~ReconnectStrategy() = default;
        virtual std::chrono::milliseconds nextDelay(int attempt) const = 0;
    };

    /**
     * @brief Constant backoff strategy used by default.
     */
    class ConstantReconnectStrategy : public ReconnectStrategy {
    public:
        explicit ConstantReconnectStrategy(std::chrono::milliseconds delay) : delay_(delay) {}
        std::chrono::milliseconds nextDelay([[maybe_unused]] int attempt) const override { return delay_; }
    private:
        std::chrono::milliseconds delay_;
    };

private:
    /// Shared pointer to Controls object for network control.
    std::shared_ptr<Controls> controls_;
    /// Shared pointer to Sensors object for sensor data.
    std::shared_ptr<Sensors> sensors_;
    /// Shared pointer to a protobuf GetMapResponse object for map data.
    std::shared_ptr<map_service::GetMapResponse> map_;

    /// Status returned by the gRPC client.
    grpc::Status clientStatus;

    /// Flag indicating whether a connection is established.
    bool connected = false;

    /// Shared pointer to Arper service instance.
    std::shared_ptr<Arper> arpService;

    /// gRPC server builder.
    grpc::ServerBuilder builder;
    /// gRPC server service instance.
    grpcServer service;
    /// Shared pointer to gRPC client instance.
    std::shared_ptr<grpcClient> clientPtr;

    /// List of pointers to control machines.
    std::vector<ControlMachine*> controlMachineAddresses;
    /// List of verified control machines.
    std::vector<ControlMachine*> verified;

    /// Last connected IP address.
    std::string lastConnectedIP;

    /// Current connection state.
    ConnectionState connection_state_{ConnectionState::Discovering};
    /// Strategy defining reconnection backoff.
    std::unique_ptr<ReconnectStrategy> reconnect_strategy_;

    void setConnectionState(ConnectionState state);
public:
    /**
     * @brief Constructs a NetworkController.
     *
     * @param controlsPtr Shared pointer to the Controls object.
     * @param sensorsPtr Shared pointer to the Sensors object.
     * @param mapPtr Shared pointer to the GetMapResponse protobuf object.
     */
    NetworkController(std::shared_ptr<Controls> controlsPtr,
                      std::shared_ptr<Sensors> sensorsPtr,
                      std::shared_ptr<map_service::GetMapResponse> mapPtr);

    /**
     * @brief Starts the ARP ping service.
     *
     * This method initializes and starts the ARP service using the provided broadcast and ARP ports.
     *
     * @param bcastPort Broadcast port number.
     * @param arpingPort ARP service port number.
     * @return int Return code from starting the ARP service.
     */
    int startArpingService(int bcastPort, int arpingPort);

    /**
     * @brief Stops the ARP ping service.
     */
    void stopArpingService();

    /**
     * @brief Runs the gRPC client with the specified server address.
     *
     * Creates a new gRPC client instance for the provided server address, extracts and stores the IP,
     * and starts separate threads for map and data stream exchange.
     *
     * @param server_address Reference to the server address string (IP and port).
     * @param tryConnectIfFailed If true, will attempt to reconnect if the connection fails.
     * @return int Return code (0 on success).
     */
    int runClient(std::string &server_address, bool tryConnectIfFailed = true); // Server address & port for client

    /**
     * @brief Runs the gRPC client using an unverified control machine.
     *
     * This overload uses the first control machine address in the list and attempts to connect.
     *
     * @param tryConnectToUnverified If true, will try connecting to an unverified control machine.
     * @return int Return code (0 on success, negative if error).
     */
    int runClient(bool tryConnectToUnverified = true); // Server address & port for client

    /**
     * @brief Runs the gRPC server.
     *
     * Starts a gRPC server that listens on the specified address mask and port.
     *
     * @param address_mask Address mask (IP and port) for the server.
     * @return int Return code (0 on success).
     */
    int runServer(std::string &address_mask); // Address mask & port for server

    /**
     * @brief Retrieves the last connected IP address.
     *
     * @return std::string Last connected IP address.
     */
    std::string getLastConnectedIP();

    /**
     * @brief Observer callback invoked when Arper discovers a control machine.
     */
    void onDiscovered(const ControlMachine &machine) override;

    /**
     * @brief Returns the gRPC client instance.
     *
     * @return std::shared_ptr<grpcClient>& Reference to the shared pointer holding the gRPC client.
     */
    std::shared_ptr<grpcClient>& getClientInstance() {
        return clientPtr;
    }
};

#endif // NETWORKCONTROLLER_H
