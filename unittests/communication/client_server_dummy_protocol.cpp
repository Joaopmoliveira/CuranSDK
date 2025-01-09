#include "communication/Client.h"
#include "communication/Server.h"
#include <gtest/gtest.h>
#include <asio.hpp>
#include "communication/ProtocolValidationHelper.h"


/*
In memory the following message with be stored with the following memory layout
  <-  8 bytes ->        <-  size_of_vector ->
[ size_of_vector ]     [    ... data ....    ]
*/
struct DummyMessage{
    std::vector<double> values;

    DummyMessage(const std::vector<double>& vals){
        values = vals;
    }

};

struct DummyProtocol;

struct DummyClientConnection
{
    std::shared_ptr<DummyMessage> message = nullptr;
    std::shared_ptr<curan::communication::Client<DummyProtocol>> owner;
    DummyClientConnection(std::shared_ptr<curan::communication::Client<DummyProtocol>> supplied_owner){

    }
};

struct DummyProtocol
{
public:
    using signature = std::function<void(const size_t &, const std::error_code &, std::shared_ptr<DummyMessage>)>;

    static void start(std::shared_ptr<curan::communication::Client<DummyProtocol>> client){

    }

    void static read_header_first_time(DummyClientConnection val){

    }

    void static read_body(DummyClientConnection val, std::error_code ec){

    }

    void static read_header(DummyClientConnection val, std::error_code ec){

    }
};

TEST(UnitTestClientServer, DummyServerClient)
{
    unsigned short port = 50000;
    asio::io_context io_context;
    asio::ip::tcp::resolver resolver(io_context);
	auto client = curan::communication::Client<DummyProtocol>::make( io_context ,resolver.resolve("localhost", std::to_string(port)));
    client->connect([](const size_t &, const std::error_code &, std::shared_ptr<DummyMessage> msg){ std::printf("received vector of size: %llu\nwith values: ",msg->values.size()); for(auto v : msg->values) std::printf("%f ",v);  });
    auto server = curan::communication::Server<DummyProtocol>::make(io_context,port);
}