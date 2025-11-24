#include "client.h"

grpcClient::grpcClient(std::shared_ptr<grpc::Channel> channel,
               std::shared_ptr<Controls> controlsPtr,
               std::shared_ptr<Sensors> sensorsPtr,
               std::shared_ptr<map_service::GetMapResponse> mapPtr,
               bool tryConnectIfFailed)
      : stub_(ClientOnRobot::NewStub(channel)),
        retryConnect(tryConnectIfFailed),
        controls_(controlsPtr),
        sensors_(sensorsPtr),
        map_(mapPtr)
{
    clientChannel = channel;
    std::cout << "Client started" << std::endl;
}

// Assembles the client's payload, sends it and presents the response back from the server.
grpc::Status grpcClient::DataExchange()
{
    grpc::Status status;
    bool retry = true; // Flag that defined need connection retry or no

    while(retry){
        // Container for the data we expect from the server.
        Controls reply;

        // Context for the client. It could be used to convey extra information to
        // the server and/or tweak certain RPC behaviors.
        grpc::ClientContext context;

        // The actual RPC.
        status = stub_->DataExchange(&context, *sensors_, &reply);

        std::unique_lock<std::mutex> ul(muClient);

        // Act upon its status.
        if (status.ok()){
          *controls_ = reply;
          retry = false;
        }
        else{
          std::cout << "DataExchange rpc failed!" << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds( retryDelayMilliseconds ));
        }

    }

    return status;
}

void grpcClient::stopStream()
{
    stoppedStream = true;
    std::cout << "Client stopped" << std::endl;
}

grpc::Status grpcClient::DataStreamExchange()
{
    grpc::Status status;
    bool retry = true;
    stoppedStream = false;

    while(retry){
        // Context for the client. It could be used to convey extra information to
        // the server and/or tweak certain RPC behaviors.
        grpc::ClientContext context;

        std::shared_ptr<grpc::ClientReaderWriter<Sensors, Controls> > stream(
            stub_->DataStreamExchange(&context));

        while(!stoppedStream && (clientChannel->GetState(true) == 2) )
        {
            std::unique_lock<std::mutex> lock(muClient);

            // Write sensors
            stream->Write(*sensors_);

            // Read controls & write to protos
            stream->Read(controls_.get());
        }

        stream->WritesDone();

        status = stream->Finish();
        if (status.ok())
            retry = false;
        else {
            std::cout << "Error " << status.error_code() << " : " << status.error_message() << std::endl;
            std::cout << "DataStreamExchange rpc failed." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds( retryDelayMilliseconds ));
        }

    }

    stoppedStream = false;

    return status;
}

grpc::Status grpcClient::MapStream()
{
    grpc::Status status;
    bool retry = true; // Flag that defined need connection retry or no
    stoppedStream = false;

    while(retry){
        map_service::GetMapRequest request;
        grpc::ClientContext context;

        {
            std::unique_lock<std::mutex> lock(muMap);
            if (!map_) {
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "No map data to send.");
            }
        }

        std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest> > stream(
            stub_->MapStream(&context));

        while(!stoppedStream && (clientChannel->GetState(true) == 2) )
        {
            // Write map
            {
                std::unique_lock<std::mutex> lock(muMap);
                stream->Write(*map_);
            }

            // Read map
            stream->Read(&request);
        }

        stream->WritesDone();

        status = stream->Finish();
        if (status.ok())
            retry = false;
        else {
            std::cout << "Error " << status.error_code() << " : " << status.error_message() << std::endl;
            std::cout << "MapStream rpc failed." << std::endl;
        }
    }

    return status;
}
