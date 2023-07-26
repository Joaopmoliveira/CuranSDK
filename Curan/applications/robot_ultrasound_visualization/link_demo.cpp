#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "link_demo.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"

bool process_image_message(std::shared_ptr<SharedRobotState> state , igtl::MessageBase::Pointer val){
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
    if(!state->dynamic_texture){
    	int x, y, z;
	    message_body->GetDimensions(x, y, z);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = y;
        infotexture.width = x;
        infotexture.spacing = {0.0001852,0.0001852,0.0001852};
        infotexture.origin = {0.0,0.0,0.0};
        infotexture.builder = vsg::Builder::create();
        state->dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
    }
    auto updateBaseTexture = [message_body](vsg::vec4Array2D& image)
    {
        int x, y, z;
	    message_body->GetDimensions(x, y, z);
        auto image_raw = (unsigned char*)message_body->GetScalarPointer();
        assert(image.width()==x && image.height()==y);
        using value_type = typename vsg::vec4Array2D::value_type;
        for (uint32_t r = 0; r < image.height(); ++r)
        {
            value_type* ptr = &image.at(0, r);
            for (size_t c = 0; c < image.width(); ++c)
            {
                ptr->r = *image_raw;
                ptr->g = *image_raw;
                ptr->b = *image_raw;
                ptr->a = 1.0f;

                ++ptr;
                ++image_raw;
            }
        }
    };
    state->dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);
    igtl::Matrix4x4 image_transform;
    message_body->GetMatrix(image_transform);
    vsg::dmat4 homogeneous_transformation;
    for(size_t row = 0 ; row < 4 ; ++row)
        for(size_t col = 0; col < 4 ; ++col)
            homogeneous_transformation(col,row) = image_transform[row][col];
    state->dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_transform(homogeneous_transformation*state->calibration_matrix);
	return true;
}


std::map<std::string,std::function<bool(std::shared_ptr<SharedRobotState> state , igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"IMAGE",process_image_message}
};

bool process_message(std::shared_ptr<SharedRobotState> state , size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	assert(val.IsNotNull());
	if (er)
        return true;

    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(state,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}


bool client_callback(std::shared_ptr<SharedRobotState> state,const size_t& loc, const std::error_code& err, std::shared_ptr<curan::communication::FRIMessage> message){
	if (err)
        return true;
    auto robotRenderableCasted = state->robot->cast<curan::renderable::SequencialLinks>();
    for(size_t link = 0 ; link< curan::communication::FRIMessage::n_joints ; ++link){
        robotRenderableCasted->set(link,message->angles[link]);
        std::printf("values: %f %f %f \n",message->angles[link],message->external_torques[link],message->measured_torques[link]);
    }
    return 0;
}


int communication(std::shared_ptr<SharedRobotState> state){
    asio::io_context context;
    curan::communication::interface_igtl igtlink_interface;
	curan::communication::Client::Info construction{ context,igtlink_interface };
	asio::ip::tcp::resolver resolver(context);
	auto endpoints = resolver.resolve("localhost", std::to_string(18944));
	construction.endpoints = endpoints;
	curan::communication::Client client{ construction };

	auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	try{
		if (process_message(state,protocol_defined_val, er, val) || state->should_kill_myself())
			context.stop();
	} catch(...)    {
		std::cout << "Exception was thrown\n";
	}
	};
	auto connectionstatus = client.connect(lam);


    curan::communication::interface_fri fri_interface;
	curan::communication::Client::Info fri_construction{ context,igtlink_interface };
	asio::ip::tcp::resolver fri_resolver(context);
	auto fri_endpoints = resolver.resolve("172.31.1.148", std::to_string(50010));
	construction.endpoints = endpoints;
	curan::communication::Client fri_client{ construction };
	auto fri_lam = [&](const size_t& loc, const std::error_code& err, std::shared_ptr<curan::communication::FRIMessage> message) {
	try{
		if (client_callback(state,loc, err, message) || state->should_kill_myself())
			context.stop();
	} catch(...)    {
		std::cout << "Exception was thrown\n";
	}
	};
    auto connections_state = fri_client.connect(fri_lam);
	context.run();
    return 0;
}