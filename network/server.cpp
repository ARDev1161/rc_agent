#include "server.h"

void grpcServer::setProtosPointers(std::shared_ptr<Controls> controlsPtr, std::shared_ptr<Sensors> sensorsPtr)
{
    this->controls_ = controlsPtr;
    this->sensors_ = sensorsPtr;
}

grpc::Status grpcServer::DataExchange ([[maybe_unused]] grpc::ServerContext* context,
                                       const Controls* request, Sensors* reply)
//                             std::shared_ptr<Controls> request,
//                             std::shared_ptr<Sensors> reply)
{
  std::unique_lock<std::mutex> ul(muServer);
  *controls_ = *request;
  ul.unlock();

  *reply = *sensors_;
  return grpc::Status::OK;
}

grpc::Status grpcServer::DataStreamExchange ([[maybe_unused]] grpc::ServerContext* context,
                                grpc::ServerReaderWriter<Sensors, Controls >* stream)
{
    std::unique_lock<std::mutex> ul(muServer, std::defer_lock);

    while(true)
    {
      ul.lock();
      if(!(stream->Read(controls_.get())))
          return grpc::Status::OK;
      ul.unlock();

      // Write controls
      stream->Write(*sensors_);
    }
}
