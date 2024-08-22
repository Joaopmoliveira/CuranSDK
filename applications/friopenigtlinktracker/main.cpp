#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include <string>
#include <csignal>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"

#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

asio::io_context context;

void signal_handler(int signal)
{
	context.stop();
	if(robot_pointer)
        robot_pointer->cancel();
}

void robot_control(curan::robotic::RobotLBR& lbr)
{
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, lbr);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && lbr)
			success = app.step();
		app.disconnect();
		std::cout << "stopping robot control!\n";
		return;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return;
	}
}

struct PlusSpecification
{
	int framerate = 30;
	std::string name;
} specification;

void start_tracking(std::shared_ptr<curan::communication::Server<curan::communication::protocols::igtlink>> server,curan::robotic::RobotLBR& lbr,std::atomic<bool>& server_has_connections)
{
	try
	{
		asio::io_context &in_context = server->get_context();
		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();

		const auto& access = lbr.atomic_acess();

		while (!in_context.stopped())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			int val = 50;
			std::string name = "empty";
			val = specification.framerate;
			name = specification.name;
			std::string device_name = "FlangeTo" + name;
			while (server_has_connections.load())
			{
				const auto start = std::chrono::high_resolution_clock::now();
				ts->GetTime();
				igtl::TrackingDataMessage::Pointer trackingMsg;
				trackingMsg = igtl::TrackingDataMessage::New();
				trackingMsg->SetDeviceName(device_name);
				igtl::TrackingDataElement::Pointer trackElement0;
				trackElement0 = igtl::TrackingDataElement::New();
				trackElement0->SetName(device_name.c_str());
				trackElement0->SetType(igtl::TrackingDataElement::TYPE_TRACKER);
				trackingMsg->AddTrackingDataElement(trackElement0);

				igtl::Matrix4x4 matrix;
				igtl::TrackingDataElement::Pointer ptr;
				trackingMsg->GetTrackingDataElement(0, ptr);
				auto state = access.load(std::memory_order_relaxed);
				for(size_t row = 0; row < 3; ++row){
					matrix[row][3] = 1000.0*state.translation[row];
					for(size_t col = 0; col < 3; ++col)
						matrix[row][col] = state.rotation[row][col];
				}
				ptr->SetMatrix(matrix);
				trackingMsg->SetTimeStamp(ts);
				trackingMsg->Pack();

				auto to_send = curan::utilities::CaptureBuffer::make_shared(trackingMsg->GetPackPointer(),trackingMsg->GetPackSize(),trackingMsg);
				server->write(to_send);

				const auto end = std::chrono::high_resolution_clock::now();
				auto val_to_sleep = std::chrono::milliseconds(val) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				auto sleep_for = (val_to_sleep.count() > 0) ? val_to_sleep : std::chrono::milliseconds(0);
				std::this_thread::sleep_for(sleep_for);
			}
		}
		std::cout << "stopping tracking!\n";
	}
	catch (...)
	{
		std::cout << "exception was thrown in the communication loop\n";
	}
}

void start_joint_tracking(std::shared_ptr<curan::communication::Server<curan::communication::protocols::fri>> server,curan::robotic::RobotLBR& lbr)
{
	asio::io_context &in_context = server->get_context();

	const auto& access = lbr.atomic_acess();

	int val = 50;
	std::string name = "empty";

	val = specification.framerate;
	name = specification.name;

	while (!in_context.stopped())
	{
		const auto start = std::chrono::high_resolution_clock::now();

		std::shared_ptr<curan::communication::FRIMessage> message = std::shared_ptr<curan::communication::FRIMessage>(new curan::communication::FRIMessage());
		auto state = access.load(std::memory_order_relaxed);
		message->angles = state.q;
		message->external_torques = state.tau_ext;
		message->measured_torques = state.tau;
		message->serialize();

		auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(),message->get_body_size() + message->get_header_size(),message);
		server->write(to_send);

		const auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0 / val)) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	std::cout << "stopping joint tracking!\n";
}

int main(int argc, char *argv[])
{
	try
	{
		// Install a signal handler
		std::signal(SIGINT, signal_handler);

		std::cout << "How to use: \nCall the executable as FRIBrigde Name_of_device Time_between_messages(milliseconds), \ne.g. : FRIBrigde Flange 40\n";
		if (argc != 3)
		{
			std::cout << "Must provide at least two arguments to the executable\n";
			//return 1;
		}
		std::string integer{argv[2]};

		size_t pos = 0;
		try
		{
			specification.framerate = std::stoi(integer, &pos);
		}
		catch (...)
		{
			return 2;
		};
		if (pos != integer.size())
		{

			std::cout << "failed to parse the supplied integer value\n";
			return 2;
		}

		specification.name = std::string(argv[1]);

		std::cout << "Outputing information with delay :" << specification.framerate << std::endl;

		unsigned short port = 50000;
		auto server = curan::communication::Server<curan::communication::protocols::igtlink>::make(context, port);
		std::atomic<bool> server_has_connections = false;
		auto callme = [&](const size_t &custom, const std::error_code &err, igtl::MessageBase::Pointer pointer)
		{
			if (err)
			{
				return;
			}
			std::cout << "Receivd message\n";
			assert(pointer.IsNotNull());
			auto temp = pointer->GetMessageType();
			if (!temp.compare("STT_TDATA"))
			{
				igtl::StartTrackingDataMessage::Pointer tracking = igtl::StartTrackingDataMessage::New();
				tracking->Copy(pointer);
				int c = tracking->Unpack(1);
				if (c & igtl::MessageHeader::UNPACK_BODY)
				{
					std::string s{tracking->GetCoordinateName()};
					int framerate = tracking->GetResolution();
					std::cout << "message coordinate name: (" << s << ") , frame rate: " << framerate << "\n";
					server_has_connections = true;
				}
				else{
					std::cout << "failed to unpack plus message\n";
					server_has_connections = false;
				}
				return;
			}
			if (!temp.compare("STP_TDATA"))
			{
				server_has_connections = true;
				std::cout << "received request to stop sending flange\n";
				return;
			}
		};

		server->connect(callme);

		std::unique_ptr<curan::robotic::HandGuidance> handguinding_controller = std::make_unique<curan::robotic::HandGuidance>();
    	curan::robotic::RobotLBR client{handguinding_controller.get(),CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};
		robot_pointer = &client;

		std::thread thred_robot_control{[&](){
			robot_control(client);
			std::cout << "robot control stopping" << std::endl;
		}}; 

		std::thread thred{[&](){
			start_tracking(server, client,server_has_connections);
			std::cout << "robot tracking stopping" << std::endl;
		}}; 

		unsigned short port_fri = 50010;

		auto server_joints = curan::communication::Server<curan::communication::protocols::fri>::make(context, port);
 
		std::thread thred_joint{[&]()
		{
			start_joint_tracking(server_joints, client);
			std::cout << "joint tracking stopping" << std::endl;
		}}; //--------------------------- 

		curan::utilities::cout << "Starting server with port: " << port << " and in the localhost\n";
		context.run();
		server_joints->cancel();
		client.cancel();
		thred_robot_control.join();
		thred_joint.join();
		thred.join();
		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
