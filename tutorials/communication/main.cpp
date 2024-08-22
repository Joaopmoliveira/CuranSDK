#include "communication_templated/Server.h"
#include "communication_templated/Client.h"
#include "communication_templated/ProtoFRI.h"

int main(){
    asio::io_context io_context;
    unsigned short port = 1000;
    auto server = curan::communication::Server<curan::communication::protocols::fri>::make(io_context,port);
    curan::communication::Client<curan::communication::protocols::fri>::Info  information{io_context};
    auto client = curan::communication::Client<curan::communication::protocols::fri>::make(information);
    return 0;
}