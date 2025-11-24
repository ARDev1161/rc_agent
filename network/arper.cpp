#include "arper.h"

Arper::Arper(IControlMachineObserver &observer, int bcastPort, int arpingPort):
    observer_(observer),
    bcastPort(bcastPort),
    arpingPort(arpingPort)
{
}

Arper::~Arper()
{
    stopArpingService();
}

int Arper::getArpMsg()
{
    arpMessage = "AMUR:";
    arpMessage += System::Info::getMachineID();
    if(arpingPort > 0)
        arpMessage += ":" + std::to_string(arpingPort);
    return 0;
}

int Arper::setSockParams(int arping_port, int bcast_port)
{
    if(bcast_port <= 0)
        bcast_port = arping_port;

    // Create broadcast socket
    bcast_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (bcast_sockfd == -1) {
        perror("Failed to create broadcast socket");
        return -1;
    }
    // Create receive socket
    recv_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sockfd == -1) {
        perror("Failed to create recieve socket");
        return -2;
    }

    int optval = 1;
    // Enable broadcast on the broadcast socket
    if (setsockopt(bcast_sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof optval) == -1) {
        perror("Failed to set broadcast socket options");
        return -3;
    }
    // Enable address reuse on the receive socket
    if (setsockopt(recv_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval) == -1) {
        perror("Failed to set recieve socket options");
        return -4;
    }

    // Configure the broadcast address and port
    std::memset(&bcast_addr, 0, sizeof bcast_addr);
    bcast_addr.sin_family = AF_INET;
    bcast_addr.sin_port = htons(bcast_port);
    bcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    // Configure the receive address and port
    std::memset(&recv_addr, 0, sizeof recv_addr);
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(arping_port);
    recv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind the receive socket to the specified address and port
    if (bind(recv_sockfd, (struct sockaddr *)&recv_addr, sizeof recv_addr) == -1) {
        perror("Failed to bind recieve socket");
        return -3;
    }

    return 0;
}

int Arper::sendBroadcastMsg(std::string &broadcastMsg)
{
    if (broadcastMsg.empty()) {
        std::cerr << "Broadcast message is empty!" << std::endl;
        return -1;
    }

    if (sendto(bcast_sockfd, broadcastMsg.c_str(), broadcastMsg.length(), 0, (struct sockaddr *)&bcast_addr, sizeof bcast_addr) == -1) {
        perror("Failed to send ARP broadcast message");
        return -2;
    }

    return 0;
}

int Arper::startArpingService(bool &connected)
{
    started = true;

    if (arpingPort <= 0 || arpingPort > 65535) {
        std::cerr << "Invalid arpingPort: " << arpingPort << std::endl;
        return -100;
    }
    if (bcastPort <= 0 || bcastPort > 65535) {
        std::cerr << "Invalid bcastPort: " << bcastPort << std::endl;
        return -101;
    }

    if(getArpMsg() != 0)
        return -1;

    if(setSockParams(arpingPort, bcastPort) != 0)
        return -2;


    // Thread to send ARP broadcast messages in a loop
    std::thread BCastSender([&]()
    {
        // Send ARP message every second if not connected
        while(!connected && started){
            sendBroadcastMsg(arpMessage);
            std::this_thread::sleep_for(1000ms);
        }
    });
    BCastSender.detach();


    // Thread to receive responses from the server
    std::thread AnswerReciever([&]()
    {
        // Wait for ARP messages while service is active
        while (started) {
            char buffer[1024];
            memset(buffer, 0, sizeof buffer);

            struct sockaddr_in control_addr;
            socklen_t control_addr_len = sizeof(control_addr);

            ssize_t recvlen = recvfrom(recv_sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&control_addr, &control_addr_len);
            if (recvlen < 0) {
                perror("Failed to receive message");
                continue;
            }

            // Process message if it have preambula - "AMUR:"
            std::string message(buffer);
            std::string separator = ":";
            size_t pos = message.find(separator);
            if("OK" == message.substr(0, pos)){
                message.erase(0, pos + separator.length());
                int curGRPCPort = stoi(message.substr(0, message.find(separator)));

                ControlMachine control(control_addr, curGRPCPort);
                control.setLastSeen(std::chrono::system_clock::now());
                observer_.onDiscovered(control);
            }
        }
    });
    AnswerReciever.detach();

    return 0;
}

void Arper::stopArpingService()
{
    started = false;

    // Close broadcast socket
    if (bcast_sockfd != -1) {
        close(bcast_sockfd);
        bcast_sockfd = -1;
    }

    // Close receive socket
    if (recv_sockfd != -1) {
        close(recv_sockfd);
        recv_sockfd = -1;
    }
}


std::chrono::system_clock::time_point ControlMachine::getLastSeen() const
{
    return last_seen;
}

void ControlMachine::setLastSeen(std::chrono::system_clock::time_point newLast_seen)
{
    last_seen = newLast_seen;
}

std::string ControlMachine::getIpAddr() const
{
    return ipAddr;
}

void ControlMachine::refreshIP()
{
    // INET6_ADDRSTRLEN is expected to be larger than INET_ADDRSTRLEN.
    // This may be necessary if IPv6 is not supported and INET6_ADDRSTRLEN is defined as 0,
    // but such cases are very unlikely.
    char s[INET6_ADDRSTRLEN > INET_ADDRSTRLEN ? INET6_ADDRSTRLEN : INET_ADDRSTRLEN] = "\0";

    switch(m_address.sin_family) {
    case AF_INET:
        inet_ntop(AF_INET, &(m_address.sin_addr), s, INET_ADDRSTRLEN);
        break;

    case AF_INET6:
        inet_ntop(AF_INET, &(m_address.sin_addr), s, INET_ADDRSTRLEN);
        break;

    default:
        std::cerr << "Unknown address family (AF)" << std::endl;
    }

    ipAddr = s;
}

int ControlMachine::grpcPort() const
{
    return m_grpcPort;
}

void ControlMachine::setGrpcPort(int newGrpcPort)
{
    m_grpcPort = newGrpcPort;
}

sockaddr_in& ControlMachine::address()
{
    return m_address;
}

const sockaddr_in& ControlMachine::address() const
{
    return m_address;
}

void ControlMachine::setAddress(const sockaddr_in &newAddress)
{
    m_address = newAddress;
}
