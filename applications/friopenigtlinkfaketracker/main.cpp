#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include <csignal>
#include "robotutils/RobotState.h"
#include "robotutils/RobotModel.h"

constexpr unsigned short DEFAULT_PORTID = 30200;

asio::io_context context;

using AtomicState = std::atomic<curan::robotic::State>;

struct PlusSpecification{
	int framerate = 30;
	std::string name;
} specification;

void signal_handler(int signal)
{
	context.stop();
}

void GetRobotConfiguration(const curan::robotic::WrappedState& wrapped,igtl::Matrix4x4& matrix)
{
	matrix[0][0] = wrapped.f_end_effector(0, 0);
	matrix[1][0] = wrapped.f_end_effector(1, 0);
	matrix[2][0] = wrapped.f_end_effector(2, 0);

	matrix[0][1] = wrapped.f_end_effector(0, 1);
	matrix[1][1] = wrapped.f_end_effector(1, 1);
	matrix[2][1] = wrapped.f_end_effector(2, 1);

	matrix[0][2] = wrapped.f_end_effector(0, 2);
	matrix[1][2] = wrapped.f_end_effector(1, 2);
	matrix[2][2] = wrapped.f_end_effector(2, 2);

	matrix[3][0] = 0.0;
	matrix[3][1] = 0.0;
	matrix[3][2] = 0.0;
	matrix[3][3] = 1.0;

	matrix[0][3] = wrapped.f_end_effector(0, 0)*1000.0;
	matrix[1][3] = wrapped.f_end_effector(1, 0)*1000.0;
	matrix[2][3] = wrapped.f_end_effector(2, 0)*1000.0;
	return;
}

void start_tracking(std::shared_ptr<curan::communication::Server<curan::communication::protocols::igtlink>> server, curan::utilities::Flag& flag, AtomicState& shared_state)
{
	try
	{
		
		asio::io_context &in_context = server->get_context();
		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();
		curan::robotic::State current_state;
		curan::robotic::RobotModel<curan::robotic::number_of_joints> robot_model{CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};

		while (!in_context.stopped())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			int val = 50;
			std::string name = "empty";
			val = specification.framerate;
			name = specification.name;
			std::string device_name = "FlangeTo" + name;
			flag.wait();
			while (flag.value())
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

				igtl::TrackingDataElement::Pointer ptr;
				trackingMsg->GetTrackingDataElement(0, ptr);
				current_state = shared_state.load();
				robot_model.update(current_state);
				curan::robotic::WrappedState wrapped{current_state};	
				igtl::Matrix4x4 matrix;
				GetRobotConfiguration(wrapped,matrix);			
				ptr->SetMatrix(matrix);
				trackingMsg->SetTimeStamp(ts);
				trackingMsg->Pack();


				auto to_send = curan::utilities::CaptureBuffer::make_shared(trackingMsg->GetPackPointer(), trackingMsg->GetPackSize(),trackingMsg);
				server->write(to_send);

				const auto end = std::chrono::high_resolution_clock::now();
				auto val_to_sleep = std::chrono::milliseconds(val) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				auto sleep_for = (val_to_sleep.count() > 0) ? val_to_sleep : std::chrono::milliseconds(0);
				std::this_thread::sleep_for(sleep_for);
			}
		}
	}
	catch (...)
	{
		curan::utilities::cout << "exception was thrown in the communication loop\n";
	}
}

void GetRobotConfiguration(std::shared_ptr<curan::communication::FRIMessage> &message, AtomicState& shared_state)
{
	auto robot_state = shared_state.load();
	for (int i = 0; i < curan::robotic::number_of_joints; i++){
		message->angles[i] = robot_state.q[i];
		message->measured_torques[i] = robot_state.tau[i];
		message->external_torques[i] = robot_state.tau_ext[i];
	}
}

void start_joint_tracking(std::shared_ptr<curan::communication::Server<curan::communication::protocols::fri>> server, curan::utilities::Flag& flag, AtomicState& shared_state)
{
	asio::io_context &in_context = server->get_context();
	
	int val = 50;
	std::string name = "empty";

	val = specification.framerate;
	name = specification.name;

	while (!in_context.stopped())
	{
		const auto start = std::chrono::high_resolution_clock::now();
		std::shared_ptr<curan::communication::FRIMessage> message = std::shared_ptr<curan::communication::FRIMessage>(new curan::communication::FRIMessage());
		GetRobotConfiguration(message, shared_state);
		message->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(), message->get_body_size() + message->get_header_size(),message);
		server->write(to_send);
		const auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0 / val)) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
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
			return 1;
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

		unsigned short port = 50000;
		auto server = curan::communication::Server<curan::communication::protocols::igtlink>::make(context, port);
		curan::utilities::Flag state_flag;
		state_flag.set(false);
		auto callme = [&](const size_t &custom, const std::error_code &err, igtl::MessageBase::Pointer pointer)
		{
			if (err)
			{
				return;
			}
			std::cout << "Receivd message\n";
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
					state_flag.set(true);
					std::cout << "message coordinate name: (" << s << ") , frame rate: " << framerate << "\n";
				}
				else
					std::cout << "failed to unpack plus message\n";
				return;
			}
			if (!temp.compare("STP_TDATA"))
			{
				std::cout << "received request to stop processing images\n";
				state_flag.set(false);
				return;
			}
		};

		server->connect(callme);

		curan::utilities::Flag robot_flag;
		robot_flag.set(true);

		AtomicState shared_state;
		std::atomic<bool> keep_going = true;
		std::thread state_updater{[&]()
								  {
									  curan::robotic::State updated_state;
									  curan::robotic::State current_state;
									  double time = 0.0;
									  while (keep_going){
										  for (auto &current_t : updated_state.tau_ext)
											  current_t = std::sin(time);
										  for (auto &current_t : updated_state.tau)
											  current_t = std::sin(time);
										  for (auto &current_t : updated_state.q)
											  current_t = std::sin(time);
										  current_state.differential(updated_state);
										  shared_state.store(current_state);
										  time += 0.016;
										  std::this_thread::sleep_for(std::chrono::milliseconds(1));
									  }
								  }};


		std::thread thred{[&](){ start_tracking(server, state_flag, shared_state);}};

		unsigned short port_fri = 50010;

		auto server_joints = curan::communication::Server<curan::communication::protocols::fri>::make(context, port_fri);

		std::thread thred_joint{[&](){ start_joint_tracking(server_joints, state_flag, shared_state);}};

		curan::utilities::cout << "Starting server with port: " << port << " and in the localhost\n";
		context.run();
		keep_going.store(false);
		robot_flag.set(false);
		thred.join();
		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
