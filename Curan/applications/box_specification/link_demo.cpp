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
#include "rendering/Box.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "itkImage.h"
#include "SharedRobotState.h"

using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<OutputPixelType, 3>;

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

        curan::renderable::Box::Info infobox;
        infobox.builder = vsg::Builder::create();
        infobox.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
        infobox.geomInfo.dx = vsg::vec3(1.0f,0.0,0.0);
        infobox.geomInfo.dy = vsg::vec3(0.0,1.0f,0.0);
        infobox.geomInfo.dz = vsg::vec3(0.0,0.0,1.0f);
        infobox.stateInfo.wireframe = true;
        infobox.geomInfo.position = vsg::vec3(0.5,0.5,0.5);
        state->caixa = curan::renderable::Box::make(infobox);
        (*state->window_pointer) << state->caixa;
    }
    auto updateBaseTexture = [message_body](vsg::vec4Array2D& image)
    {
        try{
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

        } catch(std::exception& e){
            std::cout << "exception : " << e.what() << std::endl;;
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

    OutputImageType::Pointer image_to_render;
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

    if(state->add_image_to_box_specifier){
        state->box_class.add_frame(image_to_render);
        state->box_class.update();
    }

    auto caixa = state->box_class.get_final_volume_vertices();
    vsg::dmat4 transform_matrix;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            transform_matrix(col,row) = image_to_render->GetDirection()[row][col];

    transform_matrix(3,0) = image_to_render->GetOrigin()[0];
    transform_matrix(3,1) = image_to_render->GetOrigin()[1];
    transform_matrix(3,2) = image_to_render->GetOrigin()[2];
    
    vsg::dmat3 rotation_0_1;

    vsg::dmat4 box_transform_matrix = vsg::translate(0.0,0.0,0.0);

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row){
            box_transform_matrix(col,row) = caixa.axis[col][row];
            rotation_0_1(col,row) = box_transform_matrix(col,row);
        }

    vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = caixa.center[0];
    position_of_center_in_global_frame[1] = caixa.center[1];
    position_of_center_in_global_frame[2] = caixa.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = caixa.extent[0];
    position_in_local_box_frame[1] = caixa.extent[1];
    position_in_local_box_frame[2] = caixa.extent[2]; 

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

    box_transform_matrix(3,0) = global_corner_position[0];
    box_transform_matrix(3,1) = global_corner_position[1];
    box_transform_matrix(3,2) = global_corner_position[2];
    box_transform_matrix(3,3) = 1;

    
    state->caixa->cast<curan::renderable::Box>()->set_scale(caixa.extent[0]*2,caixa.extent[1]*2,caixa.extent[2]*2);
    state->caixa->update_transform(box_transform_matrix);
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
	auto connectionstatus = client.connect(lam);
    if(!connectionstatus){
        throw std::runtime_error("missmatch between communication interfaces");
    }

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
	auto fri_connectionstatus = fri_client.connect(lam_fri);
    if(!fri_connectionstatus){
        throw std::runtime_error("missmatch between communication interfaces");
    }

    auto beatifull_button = [&](){
        std::cout << "please click any key to start defining the bounding box ...\n";
        char c;
        std::cin >> c;
        state->add_image_to_box_specifier = true;
        std::cout << "when the box is defined please click any key to stop defining the bounding box ...\n";
        std::cin >> c;
        state->add_image_to_box_specifier  = false;
    };
    std::thread box_specified_button{beatifull_button};

	context.run();
    box_specified_button.join();
    return 0;
}