#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/SafeQueue.h"

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <csignal>

constexpr unsigned short DEFAULT_PORTID = 30200;

std::atomic<bool> progress = true;

void signal_handler(int signal)
{
    progress.store(false);
}

void robot_control(std::shared_ptr<SharedState> shared_state,std::shared_ptr<curan::utilities::Flag> flag) {
	try{
	curan::utilities::cout << "Lauching robot control thread\n";
	MyLBRClient client = MyLBRClient(shared_state);
	KUKA::FRI::UdpConnection connection;
	KUKA::FRI::ClientApplication app(connection, client);
	app.connect(DEFAULT_PORTID, NULL);
	bool success = app.step();
	if(success)
		curan::utilities::cout << "Established first message between robot and host\n";
	else
	curan::utilities::cout << "Failed to established first message between robot and host\n";
	while (success && flag->value())
		success = app.step();
	app.disconnect();
	return;
	} catch(...){
		std::cout << "robot control exception\n";
		return;
	}
}

void render_robot_scene(std::atomic<std::array<double,NUMBER_OF_JOINTS>>& robot_joint_config){
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
   auto robot = curan::renderable::SequencialLinks::make(create_info);
   window << robot;
   while(progress.load()){
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		auto local = robot_joint_config.load();
		for(size_t joint_index = 0 ; joint_index < NUMBER_OF_JOINTS; ++joint_index)
        	robot->cast<curan::renderable::SequencialLinks>()->set(joint_index,local[joint_index]);
   }
};

int main(int argc, char* argv[]) {
	// Install a signal handler
    std::signal(SIGINT, signal_handler);
	try{
	std::cout << "How to use: \nCall the executable as DataCollection filename, \ne.g. : DataCollection robot_movement\n";
	if(argc!=2){
		std::cout << "Must provide at least one argument to the executable\n";
		return 1;
	}
	std::string filename{argv[1]};
	
	auto robot_flag = curan::utilities::Flag::make_shared_flag();
	robot_flag->set();

	auto shared_state = std::make_shared<SharedState>();
	shared_state->is_initialized.store(false);

	auto robot_functional_control = [shared_state,robot_flag]() {
		robot_control(shared_state, robot_flag);
	};

	std::thread thred_robot_control{ robot_functional_control };
	std::atomic<std::array<double,NUMBER_OF_JOINTS>> robot_joint_config;
	auto robot_render = [&]() {
		render_robot_scene(robot_joint_config);
	};

	std::thread thred_robot_render{ robot_render };

	//now that the control is running in parallel we can record the 
	//readings from the shared state into a structure (still undecided if it should be a file or not)

	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	double toolMass = 0.0;                                                                     // No tool for now
	Vector3d toolCOM = Vector3d::Zero(3, 1);
	Matrix3d toolInertia = Matrix3d::Zero(3, 3);
	std::unique_ptr<ToolData> myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);
	robot->attachToolToRobotModel(myTool.get());
	KUKA::FRI::LBRState state;
	while(progress.load()){
		if(shared_state->is_initialized){
			state = shared_state->robot_state;
			std::array<double,7> joint_config;
			auto _qCurr = state.getMeasuredJointPosition();
			memcpy(joint_config.data(), _qCurr, NUMBER_OF_JOINTS * sizeof(double));
			robot_joint_config.store(joint_config);
		}
	}

	robot_flag->clear();
	thred_robot_control.join();
	thred_robot_render.join();
	return 0;
	} catch(std::exception& e){
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
