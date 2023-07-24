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
        vsg::dmat4 homogeneous_transformation;
        for(size_t row = 0 ; state->calibration_matrix.rows(); ++row)
            for(size_t col = 0; state->calibration_matrix.cols(); ++col)
                homogeneous_transformation(col,row) = state->calibration_matrix(row,col);
        state->dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_transform(homogeneous_transformation);
        state->robot->append(*state->dynamic_texture);
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
	context.run();
    return 0;
}

int render(std::shared_ptr<SharedRobotState> state)
{ 
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{2000, 1200};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    state->robot = curan::renderable::SequencialLinks::make(create_info);
    window << state->robot;

    auto communication_callable = [state](){
        communication(state);
    };
    //here I should lauch the thread that does the communication and renders the image above the robotic system 
    std::thread communication_thread(communication_callable);

    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here
	
	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	// Attach tool
	double toolMass = 0.0;                                                                     // No tool for now
	RigidBodyDynamics::Math::Vector3d toolCOM = RigidBodyDynamics::Math::Vector3d::Zero(3, 1);
	RigidBodyDynamics::Math::Matrix3d toolInertia = RigidBodyDynamics::Math::Matrix3d::Zero(3, 3);
	auto myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);

	robot->attachToolToRobotModel(myTool.get());

    RigidBodyDynamics::Math::VectorNd measured_torque = RigidBodyDynamics::Math::VectorNd::Zero(7,1);
    Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
    Vector3d p_0_cur = Vector3d(0, 0, 0.045);
    RigidBodyDynamics::Math::MatrixNd Jacobian = RigidBodyDynamics::Math::MatrixNd::Zero(6, NUMBER_OF_JOINTS);

    auto robotRenderableCasted = state->robot->cast<curan::renderable::SequencialLinks>();

    while(window.run_once() && !state->should_kill_myself()) {
        auto current_reading = state->read();
        auto q_current = current_reading.getMeasuredJointPosition();
        auto tau_current = current_reading.getExternalTorque();

	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->q[i] = q_current[i];
            robotRenderableCasted->set(i,q_current[i]);
		    measured_torque[i] = tau_current[i];
	    }
        static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->qDot[i] = (q_current[i] - q_old[i]) / current_reading.getSampleTime();
	    }   

        robot->getMassMatrix(iiwa->M,iiwa->q);
	    iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,7);              // 3x1 position of flange (body = 7), expressed in base coordinates

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
    }
    communication_thread.join();
    return 0;
}
