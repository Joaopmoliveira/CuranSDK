#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include "utils/Logger.h"
#include "utils/Flag.h"

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

constexpr unsigned short DEFAULT_PORTID = 30000;


void robot_control(std::shared_ptr<SharedState> shared_state,std::shared_ptr<curan::utilities::Flag> flag) {
	MyLBRClient client = MyLBRClient(shared_state);
	KUKA::FRI::UdpConnection connection;
	KUKA::FRI::ClientApplication app(connection, client);
	app.connect(DEFAULT_PORTID, NULL);
	try
	{
		bool success = true;
		while (success && flag->value())
		{
			success = app.step();
		}
	}
	catch (std::exception& e) {
		return;
	}
	app.disconnect();
	return;
}

void GetRobotConfiguration(igtl::Matrix4x4& matrix, kuka::Robot* robot, RobotParameters* iiwa,std::shared_ptr<SharedState> shared_state)
{
	auto robot_state = shared_state->robot_state.load();
	auto _qCurr = robot_state.getMeasuredJointPosition();
	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		iiwa->q[i] = _qCurr[i];
	}
	Vector3d p_0_cur = Vector3d::Zero(3, 1);
	Matrix3d R_0_7 = Matrix3d::Zero(3, 3);
	Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
	robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, NUMBER_OF_JOINTS);              // 3x1 position of flange (body = 7), expressed in base coordinates
	robot->getRotationMatrix(R_0_7, iiwa->q, NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates

	matrix[0][0] = R_0_7(0, 0);
	matrix[1][0] = R_0_7(1, 0);
	matrix[2][0] = R_0_7(2, 0);

	matrix[0][1] = R_0_7(0, 1);
	matrix[1][1] = R_0_7(1, 1);
	matrix[2][1] = R_0_7(2, 1);

	matrix[0][2] = R_0_7(0, 2);
	matrix[1][2] = R_0_7(1, 2);
	matrix[2][2] = R_0_7(2, 2);

	matrix[3][0] = 0.0;
	matrix[3][1] = 0.0;
	matrix[3][2] = 0.0;
	matrix[3][3] = 1.0;

	matrix[0][3] = p_0_cur(0,0);
	matrix[0][3] = p_0_cur(1,0);
	matrix[0][3] = p_0_cur(2,0);
}

struct OutsideValues {
	int framerate = 30;
	std::string name;
	std::mutex mut;
};

OutsideValues values;

void start_joint_tracking(curan::communication::Server& server,std::shared_ptr<curan::utilities::Flag> flag, std::shared_ptr<SharedState> shared_state) {
	asio::io_context& context = server.get_context();

	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	while (!context.stopped()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		int val = 50;
		curan::utilities::cout << "waiting for outside value\n";
		flag->wait();
		std::string name = "empty";
		{
			std::lock_guard<std::mutex> g(values.mut);
			val = values.framerate;
			name = values.name;
		}

		while (flag->value()) {
			const auto start = std::chrono::high_resolution_clock::now();

			curan::communication::FRIMessage message;
			message.angles ;
			message.external_torques ;
			message.measured_torques ; 
			message.serialize();

			auto callable = [message]() {
				return asio::buffer(message.get_buffer(),message.get_body_size()+message.get_header_size());
			};
			auto to_send = curan::utilities::CaptureBuffer::make_shared(std::move(callable));
			server.write(to_send);

			const auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0/ val)) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
	}
}

void start_tracking(curan::communication::Server& server,std::shared_ptr<curan::utilities::Flag> flag, std::shared_ptr<SharedState> shared_state) {
	asio::io_context& context = server.get_context();
	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();
	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	while (!context.stopped()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		int val = 50;
		curan::utilities::cout << "waiting for outside value\n";
		flag->wait();
		std::string name = "empty";
		{
			std::lock_guard<std::mutex> g(values.mut);
			val = values.framerate;
			name = values.name;
		}
		std::string device_name = "FlangeTo" + name;

		curan::utilities::cout << "name to outside:" << device_name << "\nstarting communication!\n";

		igtl::TrackingDataMessage::Pointer trackingMsg;
		trackingMsg = igtl::TrackingDataMessage::New();
		trackingMsg->SetDeviceName(device_name);

		igtl::TrackingDataElement::Pointer trackElement0;
		trackElement0 = igtl::TrackingDataElement::New();
		trackElement0->SetName(device_name.c_str());
		trackElement0->SetType(igtl::TrackingDataElement::TYPE_TRACKER);

		trackingMsg->AddTrackingDataElement(trackElement0);

		static float phi0 = 0.0;
		static float theta0 = 0.0;


		while (flag->value()) {
			const auto start = std::chrono::high_resolution_clock::now();
			ts->GetTime();

			igtl::Matrix4x4 matrix;
			igtl::TrackingDataElement::Pointer ptr;

			trackingMsg->GetTrackingDataElement(0, ptr);
			GetRobotConfiguration(matrix, robot.get(), iiwa.get(), shared_state);
			ptr->SetMatrix(matrix);

			trackingMsg->Pack();

			auto callable = [trackingMsg]() {
				return asio::buffer(trackingMsg->GetPackPointer(), trackingMsg->GetPackSize());
			};
			auto to_send = curan::utilities::CaptureBuffer::make_shared(std::move(callable));
			server.write(to_send);

			const auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0/ val)) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
	}
}

int main() {
	unsigned short port_igtl = 50000;
	unsigned short port_fri = 50010;
	asio::io_context io_context;
	curan::communication::interface_igtl igtlink_interface;
	curan::communication::Server::Info construction{ io_context,igtlink_interface ,port_igtl };
	curan::communication::Server server{ construction };
	
	auto state_flag = curan::utilities::Flag::make_shared_flag();
	auto robot_flag = curan::utilities::Flag::make_shared_flag();
	robot_flag->set();

	auto shared_state = std::make_shared<SharedState>();
	curan::communication::interface_igtl callme = [state_flag,&server](const size_t& custom, const std::error_code& err, igtl::MessageBase::Pointer pointer) {
		if (err) {
			return;
		}
		curan::utilities::cout << "message received!\n";
		auto temp = pointer->GetMessageType();
		curan::utilities::cout << "the message type is:" << temp << "\n";
		if (!temp.compare("STT_TDATA")) {
			igtl::StartTrackingDataMessage::Pointer tracking = igtl::StartTrackingDataMessage::New();
			tracking->Copy(pointer);
			int c = tracking->Unpack(1);
			if (c & igtl::MessageHeader::UNPACK_BODY) {
				std::string s{ tracking->GetCoordinateName() };
				int framerate = tracking->GetResolution();
				curan::utilities::cout << "message coordinate name: (" << s << ") , frame rate: " << framerate << "\n";
				{
					std::lock_guard<std::mutex> g(values.mut);
					values.framerate = framerate;
					values.name = s;
					state_flag->set();
				}
			}
			return;
		}
		if (!temp.compare("STP_TDATA")) {
			curan::utilities::cout << "received request to stop processing images\n";
			state_flag->clear();
			server.get_context().stop();
			return;
		}
	};

	auto val = server.connect(callme);
	if (!val)
		return 1;

	auto openigtlink_tracking = [state_flag, &server, shared_state]() {
		start_tracking(server, state_flag, shared_state);
	};

	curan::utilities::cout << "Starting server with port: " << port_igtl << " and in the localhost\n";


	std::thread thred{ openigtlink_tracking };
	auto robot_functional_control = [shared_state, robot_flag]() {
		robot_control(shared_state, robot_flag);
	};
	std::thread thred_robot_control{ robot_functional_control };

	curan::communication::interface_fri fri_interface;
	curan::communication::Server::Info construction_joints{ io_context,fri_interface ,port_fri };
	curan::communication::Server server_joints{ construction_joints };

	auto joint_tracking = [](){

	};

	curan::utilities::cout << "Starting server with port: " << port_fri << " and in the localhost\n";

	io_context.run();
	robot_flag->clear();
	thred_robot_control.join();
	thred.join();
	return 0;
}