#include "dummy_server.h"



struct SharedState{
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> texture;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context) : window{in_window},context{in_context} {}
};

std::unique_ptr<SharedState> shared_state = nullptr;

void process_transform_message(igtl::MessageBase::Pointer received_transform){
    std::cout << "received transform message" << std::endl;
    igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(received_transform);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY) && !shared_state->texture)
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    igtl::Matrix4x4 local_mat;
	transform_message->GetMatrix(local_mat);
    vsg::dmat4 transformmat;

    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat[row][col] = local_mat[row][col];

    shared_state->texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat);
}

void process_image_message(igtl::MessageBase::Pointer received_transform){
    std::cout << "received image message" << std::endl;
    igtl::ImageMessage::Pointer imageMessage = igtl::ImageMessage::New();
	imageMessage->Copy(received_transform);
	int c = imageMessage->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    if(!shared_state->texture){
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = height;
        infotexture.width = width;
        infotexture.builder = vsg::Builder::create();
        shared_state->texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state->window << *shared_state->texture;
    }
    shared_state->texture->cast<curan::renderable::DynamicTexture>()->update_texture([imageMessage](vsg::vec4Array2D& image)
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

std::map<std::string,std::function<void(igtl::MessageBase::Pointer)>> functions{
    {"TRANSFORM",process_transform_message},
    {"IMAGE",process_image_message}
};

void bar(size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	std::cout << "received message\n";
	assert(val.IsNotNull());
    assert(shared_state!=nullptr);
	if (er){
        shared_state->context.stop();
        return;
    } 
    if (auto search = functions.find(val->GetMessageType()); search != functions.end())
        search->second(val);
    else
        std::cout << "No functionality for function received\n";
    return;
}

void disp(asio::io_context& io_context){
try{  
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
    shared_state = std::unique_ptr<SharedState>(new SharedState{window,io_context});
    window.run();
}  catch(std::exception & e){
    std::cout << "display failure: " << e.what() << std::endl;
}
io_context.stop();
}

int main() {
try{
    std::atomic<bool> server_continue_running = true;
    asio::io_context io_context;
    unsigned short port = 18944;
    std::thread display_thread{[&](){disp(io_context);}};
    std::thread server_thread{[&](){foo(port,server_continue_running,io_context);}}; 
	curan::communication::interface_igtl igtlink_interface;
	curan::communication::Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	curan::communication::Client client{ construction };
	auto connectionstatus = client.connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val){bar(protocol_defined_val,er,val);});
	auto val = io_context.run();
    server_continue_running.store(false);
    display_thread.join();
    server_thread.join();
	return 0;
}  catch(std::exception & e){
    std::cout << "communication failure: " << e.what() << std::endl;
}
}