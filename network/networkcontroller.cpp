#include "networkcontroller.h"


NetworkController::NetworkController(std::shared_ptr<Controls> controlsPtr,
                                     std::shared_ptr<Sensors> sensorsPtr,
                                     std::shared_ptr<map_service::GetMapResponse> mapPtr) :
    controls_(controlsPtr),
    sensors_(sensorsPtr),
    map_(mapPtr)
{
}

int NetworkController::startArpingService(int bcastPort, int arpingPort)
{
    if(!arpService)
        arpService = std::make_shared<Arper>(controlMachineAddresses, bcastPort, arpingPort);
    std::cout << "Arping service started" << std::endl;

    return arpService->startArpingService(connected);
}

void NetworkController::stopArpingService()
{
    arpService->stopArpingService();
    std::cout << "Arping service stopped" << std::endl;
}

std::string NetworkController::getLastConnectedIP()
{
    return lastConnectedIP;
}

int NetworkController::runClient(std::string &server_address, bool tryConnectIfFailed)
{
    clientPtr = std::make_shared<grpcClient>(
        grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()),
        controls_,
        sensors_,
        map_
    );

    size_t pos = server_address.find(':');
    lastConnectedIP = server_address.substr(0, pos);

    std::thread mapThr([&, this]()
     {
        this->clientStatus = clientPtr->MapStream();
        if(clientStatus.ok())
            connected = true;
     }
    );
    mapThr.detach();

    std::thread dataThr([&, this]()
     {
        clientStatus = this->clientPtr->DataStreamExchange();
        if(clientStatus.ok())
            connected = true;
     }
    );
    dataThr.detach();


    return 0;
}

int NetworkController::runClient(bool tryConnectToUnverified)
{
    if(controlMachineAddresses.empty())
        return -1;

    ControlMachine *cur = controlMachineAddresses[0];
    std::string curAddr = cur->getIpAddr();
    controlMachineAddresses.erase( controlMachineAddresses.begin() );

    if(curAddr == "\0")
        return -2;

    curAddr += ":" + std::to_string(cur->grpcPort());

    std::cout << "Connecting to - " << curAddr << std::endl;
    return runClient(curAddr, true);
}

int NetworkController::runServer(std::string &address_mask)
{
    std::thread thr([&]()
     {
      grpc::EnableDefaultHealthCheckService(true);
      grpc::reflection::InitProtoReflectionServerBuilderPlugin();

      // Send protos pointers to server
      service.setProtosPointers(controls_, sensors_);

      // Listen on the given address without any authentication mechanism.
      builder.AddListeningPort(address_mask, grpc::InsecureServerCredentials());

      // Register "service" as the instance through which we'll communicate with
      // clients. In this case it corresponds to an *synchronous* service.
      builder.RegisterService(&service);

      // Finally assemble the server.
      std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
      std::cout << "Server listening on " << address_mask << std::endl;

      // Wait for the server to shutdown. Note that some other thread must be
      // responsible for shutting down the server for this call to ever return.
      server->Wait();

      std::cout << "Server stopped " << std::endl;
     }
    );
    thr.detach();

    return 0;
}
