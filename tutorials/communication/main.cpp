#include "communication/Server.h"
#include "communication/Client.h"
#include "communication/ProtoFRI.h"

int main(){
    asio::io_context io_context;
    unsigned short port = 18944; 
    auto server = curan::communication::Server<curan::communication::protocols::fri>::make(io_context,port);
    asio::ip::tcp::resolver resolver(io_context);
	auto client = curan::communication::Client<curan::communication::protocols::fri>::make(io_context,resolver.resolve("localhost", std::to_string(port)));
    return 0;
}