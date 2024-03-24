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
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"

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
        (*state->window_pointer) << *state->dynamic_texture;

        int clip_origin_x = (int)0;
        int clip_origin_y = (int)0;

        curan::image::Clipping desired_clip;
        desired_clip.clipRectangleOrigin[0] = clip_origin_x;
        desired_clip.clipRectangleOrigin[1] = clip_origin_y;
        desired_clip.clipRectangleSize[0] = x;
        desired_clip.clipRectangleSize[1] = y-15;
        state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_clipping(desired_clip);
    }
    auto updateBaseTexture = [message_body](vsg::vec4Array2D& image)
    {
            int x, y, z;
	        message_body->GetDimensions(x, y, z);
            unsigned char* scaller_buffer = (unsigned char*)message_body->GetScalarPointer();
            
            for (size_t r = 0; r < image.height(); ++r)
                {
                    using value_type = typename vsg::vec4Array2D::value_type;
                    value_type* ptr = &image.at(0, r);
                    for (size_t c = 0; c < image.width(); ++c)
                    {
                        auto val = *scaller_buffer/255.0;
                        ptr->r = val;
                        ptr->g = val;
                        ptr->b = val;
                        ptr->a = 1.0f;
                        ++ptr;
                        ++scaller_buffer;
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

    homogeneous_transformation(3,0) *= 1e-3;
    homogeneous_transformation(3,1) *= 1e-3;
    homogeneous_transformation(3,2) *= 1e-3;
    auto product = homogeneous_transformation*state->calibration_matrix;
    state->dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_transform(product);

    curan::image::InternalImageType::Pointer image_to_render;
    curan::image::igtl2ITK_im_convert(message_body, image_to_render);
    const itk::SpacePrecisionType spacing[3] = {0.0001852,0.0001852,0.0001852};
    image_to_render->SetSpacing(spacing);

    itk::Matrix<double,3,3> itk_matrix;
    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            itk_matrix(row,col) = product(col,row);

    image_to_render->SetDirection(itk_matrix);
    auto origin = image_to_render->GetOrigin();
    origin[0] = product(3,0);
    origin[1] = product(3,1);
    origin[2] = product(3,2);
    image_to_render->SetOrigin(origin);
    state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_frame(image_to_render);
	return true;
}

std::map<std::string,std::function<bool(std::shared_ptr<SharedRobotState> state , igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"IMAGE",process_image_message}
};

bool process_message(std::shared_ptr<SharedRobotState> state , size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	assert(val.IsNotNull());
	if (er)
        return true;
    std::cout << "Received message type: " << val->GetMessageType() << "\n";
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(state,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

bool process_joint_message(std::shared_ptr<SharedRobotState> state,const size_t& protocol_defined_val, const std::error_code& er , std::shared_ptr<curan::communication::FRIMessage> message){
    if (er)
        return true;
    for(size_t joint_index = 0 ; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
        state->robot->cast<curan::renderable::SequencialLinks>()->set(joint_index,message->angles[joint_index]);
    return false;
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
	client.connect(lam);

    curan::communication::interface_fri fri_interface;
	curan::communication::Client::Info fri_construction{ context,fri_interface };
	asio::ip::tcp::resolver fri_resolver(context);
	auto fri_endpoints = fri_resolver.resolve("172.31.1.148", std::to_string(50010));
	fri_construction.endpoints = fri_endpoints;
	curan::communication::Client fri_client{ fri_construction };

	auto lam_fri = [&](const size_t& protocol_defined_val, const std::error_code& er , std::shared_ptr<curan::communication::FRIMessage> message) {
	try{
		if (process_joint_message(state,protocol_defined_val, er, message))
			context.stop();
	} catch(...)    {
		std::cout << "Exception was thrown\n";
	}
	};
	fri_client.connect(lam_fri);

    auto reconstruction_sequence = [&](){
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(8);
        while(!context.stopped()){
            state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
        }
    };
    std::thread volume_reconstruction_thead(reconstruction_sequence);

	context.run();
    volume_reconstruction_thead.join();
    return 0;
}