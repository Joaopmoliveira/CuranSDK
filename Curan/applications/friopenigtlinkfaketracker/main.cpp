#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include <csignal>
#include "SharedState.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"

constexpr unsigned short DEFAULT_PORTID = 30200;

asio::io_context context;

void signal_handler(int signal)
{
	context.stop();
}

void GetRobotConfiguration(igtl::Matrix4x4 &matrix, kuka::Robot *robot, RobotParameters *iiwa, std::shared_ptr<SharedState> shared_state)
{
	static auto t1 = std::chrono::steady_clock::now();
	static double _qOld[LBR_N_JOINTS];
	auto robot_state = shared_state->robot_state.load();
	auto _qCurr = robot_state.joint_config;
	// curan::utils::cout << "the joints are: \n";
	for (int i = 0; i < LBR_N_JOINTS; i++)
	{
		iiwa->q[i] = _qCurr[i];
	}
	auto t2 = std::chrono::steady_clock::now();
	auto sampleTimeChrono = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	double sampleTime = (sampleTimeChrono.count() < 1) ? 0.001 : sampleTimeChrono.count() / 1000.0;
	t1 = t2;
	for (int i = 0; i < LBR_N_JOINTS; i++)
	{
		iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
	}

	static Vector3d p_0_cur = Vector3d::Zero(3, 1);
	static Matrix3d R_0_7 = Matrix3d::Zero(3, 3);
	static Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric

	robot->getMassMatrix(iiwa->M, iiwa->q);
	iiwa->M(6, 6) = 45 * iiwa->M(6, 6); // Correct mass of last body to avoid large accelerations
	iiwa->Minv = iiwa->M.inverse();
	robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
	robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7); // 3x1 position of flange (body = 7), expressed in base coordinates
	robot->getRotationMatrix(R_0_7, iiwa->q, LBR_N_JOINTS);		// 3x3 rotation matrix of flange, expressed in base coordinates

	p_0_cur *= 1000;
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

	matrix[0][3] = p_0_cur(0, 0);
	matrix[1][3] = p_0_cur(1, 0);
	matrix[2][3] = p_0_cur(2, 0);
}

struct PlusSpecification
{
	int framerate = 30;
	std::string name;
} specification;

void start_tracking(curan::communication::Server &server, curan::utilities::Flag& flag, std::shared_ptr<SharedState> shared_state)
{
	try
	{
		asio::io_context &in_context = server.get_context();
		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();
		// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
		kuka::Robot::robotName myName(kuka::Robot::LBRiiwa); // Select the robot here

		auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
		auto iiwa = std::make_unique<RobotParameters>();	// myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

		double toolMass = 0.0; // No tool for now
		Vector3d toolCOM = Vector3d::Zero(3, 1);
		Matrix3d toolInertia = Matrix3d::Zero(3, 3);
		std::unique_ptr<ToolData> myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);
		robot->attachToolToRobotModel(myTool.get());

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

				igtl::Matrix4x4 matrix;
				igtl::TrackingDataElement::Pointer ptr;
				trackingMsg->GetTrackingDataElement(0, ptr);
				GetRobotConfiguration(matrix, robot.get(), iiwa.get(), shared_state);
				ptr->SetMatrix(matrix);
				trackingMsg->SetTimeStamp(ts);
				trackingMsg->Pack();


				auto to_send = curan::utilities::CaptureBuffer::make_shared(trackingMsg->GetPackPointer(), trackingMsg->GetPackSize(),trackingMsg);
				server.write(to_send);

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

void GetRobotConfiguration(std::shared_ptr<curan::communication::FRIMessage> &message, std::shared_ptr<SharedState> shared_state)
{
	auto robot_state = shared_state->robot_state.load();
	auto _qCurr = robot_state.joint_config;
	auto _eExtern = robot_state.external_torques;
	auto _eMeasured = robot_state.measured_torques;
	for (int i = 0; i < LBR_N_JOINTS; i++)
	{
		message->angles[i] = _qCurr[i];
		message->measured_torques[i] = _eMeasured[i];
		message->external_torques[i] = _eExtern[i];
	}
}

void start_joint_tracking(curan::communication::Server &server, curan::utilities::Flag& flag, std::shared_ptr<SharedState> shared_state)
{
	asio::io_context &in_context = server.get_context();

	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa); // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>();	// myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...
	int val = 50;
	std::string name = "empty";

	val = specification.framerate;
	name = specification.name;

	while (!in_context.stopped())
	{

		const auto start = std::chrono::high_resolution_clock::now();

		std::shared_ptr<curan::communication::FRIMessage> message = std::shared_ptr<curan::communication::FRIMessage>(new curan::communication::FRIMessage());

		GetRobotConfiguration(message, shared_state);
		//std::printf("Sending robot configuration: ");
		//for(const auto& val : shared_state->robot_state.load().joint_config)
		//	std::printf(" %f ",val);
		//std::printf("\n");
		message->serialize();

		auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(), message->get_body_size() + message->get_header_size(),message);
		server.write(to_send);

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
		curan::communication::interface_igtl igtlink_interface;
		curan::communication::Server::Info construction{context, igtlink_interface, port};
		curan::communication::Server server{construction};
		curan::utilities::Flag state_flag;
		state_flag.set(false);
		curan::communication::interface_igtl callme = [&](const size_t &custom, const std::error_code &err, igtl::MessageBase::Pointer pointer)
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

		auto val = server.connect(callme);

		curan::utilities::Flag robot_flag;
		robot_flag.set(true);

		auto shared_state = std::make_shared<SharedState>();
		std::atomic<bool> keep_going = true;
		std::thread state_updater{[&]()
								  {
									  State current_state;
									  double time = 0.0;
									  while (keep_going){
										  for (auto &current_t : current_state.external_torques)
											  current_t = std::sin(time);
										  for (auto &current_t : current_state.measured_torques)
											  current_t = std::sin(time);
										  for (auto &current_t : current_state.joint_config)
											  current_t = std::sin(time);
										  shared_state->robot_state.store(current_state);
										  time += 0.016;
										  std::this_thread::sleep_for(std::chrono::milliseconds(1));
									  }
								  }};

		auto state_machine = [&state_flag, &server, shared_state]()
		{
			start_tracking(server, state_flag, shared_state);
		};

		std::thread thred{state_machine};

		unsigned short port_fri = 50010;

		curan::communication::interface_fri fri_interface;
		curan::communication::Server::Info construction_joints{context, fri_interface, port_fri};
		curan::communication::Server server_joints{construction_joints};

		auto joint_tracking = [&state_flag, &server_joints, shared_state]()
		{
			start_joint_tracking(server_joints, state_flag, shared_state);
		};
		std::thread thred_joint{joint_tracking};

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
