#include "client.h"

namespace {

template<typename StreamT, typename FactoryFn, typename PumpFn>
grpc::Status runStreamWithRetry(const std::string &stream_name,
                                bool &stopped_flag,
                                int retry_delay_ms,
                                FactoryFn &&stream_factory,
                                PumpFn &&pump_loop)
{
    grpc::Status status;
    bool retry = true;
    stopped_flag = false;

    while (retry) {
        grpc::ClientContext context;
        auto stream = stream_factory(context);
        if (!stream) {
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "Stream factory returned null");
        }

        pump_loop(*stream);

        stream->WritesDone();
        status = stream->Finish();
        if (status.ok()) {
            retry = false;
        } else {
            std::cout << "Error " << status.error_code() << " : " << status.error_message() << std::endl;
            std::cout << stream_name << " rpc failed." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
        }
    }

    return status;
}

} // namespace

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
    return runStreamWithRetry<grpc::ClientReaderWriter<Sensors, Controls>>(
        "DataStreamExchange",
        stoppedStream,
        retryDelayMilliseconds,
        [this](grpc::ClientContext &context) {
            return std::shared_ptr<grpc::ClientReaderWriter<Sensors, Controls>>(
                stub_->DataStreamExchange(&context));
        },
        [this](grpc::ClientReaderWriter<Sensors, Controls> &stream) {
            while(!stoppedStream && (clientChannel->GetState(true) == 2))
            {
                std::unique_lock<std::mutex> lock(muClient);

                stream.Write(*sensors_);
                stream.Read(controls_.get());
            }
        }
    );
}

grpc::Status grpcClient::MapStream()
{
    return runStreamWithRetry<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(
        "MapStream",
        stoppedStream,
        retryDelayMilliseconds,
        [this](grpc::ClientContext &context) {
            std::unique_lock<std::mutex> lock(muMap);
            if (!map_) {
                return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(nullptr);
            }
            return std::shared_ptr<grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest>>(
                stub_->MapStream(&context));
        },
        [this](grpc::ClientReaderWriter<map_service::GetMapResponse, map_service::GetMapRequest> &stream) {
            map_service::GetMapRequest request;

            while(!stoppedStream && (clientChannel->GetState(true) == 2))
            {
                {
                    std::unique_lock<std::mutex> lock(muMap);
                    stream.Write(*map_);
                }

                stream.Read(&request);
            }
        }
    );
}
