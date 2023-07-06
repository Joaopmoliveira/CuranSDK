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
    std::optional<vsg::ref_ptr<curan::renderable::DynamicTexture>> texture;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context) : window{in_window},context{in_context} {}
};

void process_transform_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(received_transform);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY) && !shared_state.texture)
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    igtl::Matrix4x4 local_mat;
	transform_message->GetMatrix(local_mat);
    vsg::dmat4 transformmat;

    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat[row][col] = local_mat[row][col];

    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat);
}

void process_image_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    igtl::ImageMessage::Pointer imageMessage = igtl::ImageMessage::New();
	imageMessage->Copy(received_transform);
	int c = imageMessage->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    if(!shared_state.texture){
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = height;
        infotexture.width = width;
        infotexture.builder = vsg::Builder::create();
        shared_state.texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state.window << *shared_state.texture;
    }
    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_texture([imageMessage](vsg::vec4Array2D& image)
    {
        unsigned char* data = static_cast<unsigned char*>(imageMessage->GetScalarPointer());
        size_t offset = 0;
        for(auto iterator = image.begin() ; iterator!=image.end() && offset<imageMessage->GetImageSize() ; ++iterator,++offset){
            iterator->a = 1.0; 
            iterator->r = *(data+offset); 
            iterator->g = *(data+offset); 
            iterator->b = *(data+offset); 
        }
        return;
    }
    );
}

std::map<std::string,std::function<void(SharedState&,igtl::MessageBase::Pointer)>> functions{
    {"TRANSFORM",process_transform_message},
    {"IMAGE",process_image_message}
};

void bar(SharedState& shared_state ,size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	curan::utilities::cout << "received message";
	assert(val.IsNotNull());
	if (er){
        shared_state.context.stop();
        return;
    } 
    if (auto search = functions.find(val->GetMessageType()); search != functions.end())
        search->second(shared_state,val);
    else
        std::cout << "No functionality for function received\n";
    return;
}

void communication(SharedState& state){
	curan::utilities::cout << "started running";
	unsigned short port = 18944;
	asio::io_context io_context;
	curan::communication::interface_igtl igtlink_interface;
	curan::communication::Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	curan::communication::Client client{ construction };
	auto connectionstatus = client.connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val){bar(state,protocol_defined_val,er,val);});
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
    SharedState state{window,io_context};
    std::thread communication_thread{[&](){communication(state);}};
    window.run();
    io_context.stop();
    communication_thread.join();
	return 0;
}