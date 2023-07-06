#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <iostream>
#include <optional>
#include <map>

struct SharedState{
    std::optional<vsg::ref<curan::renderable::DynamicTexture>> texture;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context) : window{in_window},context{in_context} {

    }
};

std::map<std::string,std::function<void(SharedState&,igtl::MessageBase::Pointer)>> functions{
    {"TRANSFORM",[](SharedState& state,igtl::MessageBase::Pointer){}},
    {"IMAGE",[](SharedState& state,igtl::MessageBase::Pointer){}}
};

void bar(SharedState& shared_state ,size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	curan::utilities::cout << "received message";
	assert(val.IsNotNull());
	if (er){
        shared_state.context.stop();
        return;
    } 
	std::string tmp = val->GetMessageType();

    if (auto search = functions.find(tmp); search != example.end())
        search->second(shared_state,val);
    else
        std::cout << "No functionality for function received\n";
    return;
}

void communication(SharedState& state){
	curan::utilities::cout << "started running";
	unsigned short port = 18944;
	asio::io_context io_context;
	curan::communicationinterface_igtl igtlink_interface;
	curan::communication::Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	curan::communicatio::Client client{ construction };
	auto connectionstatus = client.connect([&]((size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val)){bar(state,protocol_defined_val,er,val);});
	auto val = io_context.run();
	curan::utilities::cout << "stopped running";
}

int main() {
    asio::io_context io_context;
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};
    SharedState state{window};
    std::thread communication_thread{[&](){communication(state)}};
    window.run();
    io_context.stop();
    communication_thread.join();
	return 0;
}