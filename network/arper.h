#ifndef ARPER_H
#define ARPER_H

#include <thread>
#include <chrono>
#include <vector>
#include <cstring>

#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "../system/systeminfo.h"

using namespace std::chrono_literals;

/**
 * @brief Represents a control machine discovered via Arping.
 *
 * The ControlMachine class encapsulates the network address, IP address,
 * last seen timestamp, and gRPC port for a discovered control machine.
 */
class ControlMachine
{
    /// Structure containing the socket address information
    struct sockaddr_in m_address;
    /// IP address as a string
    std::string ipAddr;
    /// Timestamp indicating when the machine was last seen
    std::chrono::system_clock::time_point last_seen;
    /// gRPC port number of the control machine
    int m_grpcPort;

public:
    /**
     * @brief Constructs a new ControlMachine object.
     *
     * @param address The socket address of the control machine.
     * @param grpcPort The gRPC port number.
     */
    ControlMachine(const sockaddr_in &address, int grpcPort)
        : m_address(address), m_grpcPort(grpcPort)
    {
        refreshIP();
    }

    /**
     * @brief Returns the gRPC port number.
     *
     * @return int The gRPC port.
     */
    int grpcPort() const;

    /**
     * @brief Sets the gRPC port number.
     *
     * @param newGrpcPort The new gRPC port.
     */
    void setGrpcPort(int newGrpcPort);

    /**
     * @brief Returns a reference to the socket address.
     *
     * @return sockaddr_in& Reference to the socket address.
     */
    sockaddr_in& address();

    /**
     * @brief Sets the socket address.
     *
     * @param newAddress The new socket address.
     */
    void setAddress(const sockaddr_in &newAddress);

    /**
     * @brief Gets the last seen timestamp.
     *
     * @return std::chrono::system_clock::time_point Timestamp of last detection.
     */
    std::chrono::system_clock::time_point getLastSeen() const;

    /**
     * @brief Sets the last seen timestamp.
     *
     * @param newLast_seen The new timestamp.
     */
    void setLastSeen(std::chrono::system_clock::time_point newLast_seen);

    /**
     * @brief Gets the IP address as a string.
     *
     * @return std::string The IP address.
     */
    std::string getIpAddr() const;

    /**
     * @brief Refreshes the IP address from the socket address.
     */
    void refreshIP();
};


/**
 * @brief Arping service for discovering control machines.
 *
 * The Arper class provides functionality to start and stop an Arping service.
 * It sends broadcast messages to inform control computers about the robot and listens
 * for responses to update the list of control machine addresses.
 */
class Arper
{
    /// Flag indicating whether the ARP service has been started
    bool started = false;
    /// Socket file descriptor for broadcasting
    int bcast_sockfd = -1;
    /// Socket file descriptor for receiving responses
    int recv_sockfd = -1;
    /// Broadcast address structure
    struct sockaddr_in bcast_addr;
    /// Receiving address structure
    struct sockaddr_in recv_addr;

    /// Broadcast port number
    int bcastPort;
    /// ARP ping port number
    int arpingPort;
    /// Message to be sent in the ARP broadcast
    std::string arpMessage;

    /// Reference to the vector of pointers to discovered control machines
    std::vector<ControlMachine*> &controlMachineAddresses;

    /**
     * @brief Sends a broadcast ARP message.
     *
     * @param broadcastMsg The message to broadcast.
     * @return int 0 on success, negative value on error.
     */
    int sendBroadcastMsg(std::string &broadcastMsg);

    /**
     * @brief Prepares the ARP message.
     *
     * Constructs the ARP message using a fixed prefix and the system's machine ID.
     *
     * @return int 0 on success.
     */
    int getArpMsg();

    /**
     * @brief Sets the parameters for the broadcast and receiving sockets.
     *
     * Configures socket options and binds the receiving socket to the specified ARP port.
     *
     * @param arping_port ARP ping port.
     * @param bcast_port Broadcast port.
     * @return int 0 on success, negative value on error.
     */
    int setSockParams(int arping_port, int bcast_port);

public:
    /**
     * @brief Constructs a new Arper object.
     *
     * @param controlMachineAddresses Reference to the vector of ControlMachine pointers.
     * @param bcastPort Broadcast port number.
     * @param arpingPort ARP ping port number.
     */
    Arper(std::vector<ControlMachine*> &controlMachineAddresses, int bcastPort, int arpingPort);

    /**
     * @brief Destructor for Arper.
     *
     * Closes open sockets.
     */
    ~Arper();

    /**
     * @brief Starts the ARP ping service.
     *
     * Begins broadcasting ARP messages to inform control computers about the robot.
     * The service runs in separate threads for sending and receiving messages.
     *
     * @param connected Reference to a boolean flag indicating connection status.
     * @return int 0 on success, negative error code on failure.
     */
    int startArpingService(bool &connected);

    /**
     * @brief Stops the ARP ping service.
     *
     * Stops the ARP service and closes the sockets.
     */
    void stopArpingService();
};

#endif // ARPER_H
