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

bool process_joint_message(std::shared_ptr<SharedRobotState> state,const size_t& protocol_defined_val, const std::error_code& er , std::shared_ptr<curan::communication::FRIMessage> message){
    if (er)
        return true;
    for(size_t joint_index = 0 ; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
        state->robot->cast<curan::renderable::SequencialLinks>()->set(joint_index,message->angles[joint_index]);
    return false;
}

int communication(std::shared_ptr<SharedRobotState> state){
    asio::io_context context;
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

	context.run();
    return 0;
}