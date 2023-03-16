#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>

void GetRandomTestMatrix(igtl::Matrix4x4& matrix)
{
	float position[3];
	float orientation[4];

	// random position
	static float phi = 0.0;
	position[0] = 50.0 * cos(phi);
	position[1] = 50.0 * sin(phi);
	position[2] = 50.0 * cos(phi);
	phi = phi + 0.2;

	// random orientation
	static float theta = 0.0;
	orientation[0] = 0.0;
	orientation[1] = 0.6666666666 * cos(theta);
	orientation[2] = 0.577350269189626;
	orientation[3] = 0.6666666666 * sin(theta);
	theta = theta + 0.1;

	//igtl::Matrix4x4 matrix;
	igtl::QuaternionToMatrix(orientation, matrix);

	matrix[0][3] = position[0];
	matrix[1][3] = position[1];
	matrix[2][3] = position[2];
}

namespace
{
	volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int signal)
{
	gSignalStatus = signal;
}

void foo(asio::io_context& cxt, short port) {
	using namespace curan::communication;
	interface_igtl igtlink_interface;
	Server::Info construction{ cxt,igtlink_interface ,port };
	Server server{ construction };

	igtl::TransformMessage::Pointer transMsg;
	transMsg = igtl::TransformMessage::New();
	transMsg->SetDeviceName("Tracker");

	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();

	while (!gSignalStatus) {
		auto start = std::chrono::high_resolution_clock::now();
		igtl::Matrix4x4 matrix;
		GetRandomTestMatrix(matrix);
		ts->GetTime();
		transMsg->SetMatrix(matrix);
		transMsg->SetTimeStamp(ts);
		transMsg->Pack();
		auto callable = [transMsg]() {
			return asio::buffer(transMsg->GetPackPointer(), transMsg->GetPackSize());
		};
		auto to_send = curan::utils::CaptureBuffer::make_shared(std::move(callable));
		server.write(to_send);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::seconds(1) - std::chrono::duration_cast<std::chrono::seconds>(end - start));
	}
	cxt.stop();
}

void bar(size_t protocol_defined_val, std::system_error er, igtl::MessageBase::Pointer val) {
	if (val->GetMessageType() == "TRANSFORM") {
		std::cout << "Receiving TRANSFORM data type\n";
		igtl::TransformMessage::Pointer transMsg = igtl::TransformMessage::New();

		transMsg->Copy(val);

		// Deserialize the transform data
		// If you want to skip CRC check, call Unpack() without argument.
		int c = transMsg->Unpack(1);

		if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
		{
			// Retrive the transform data
			igtl::Matrix4x4 matrix;
			transMsg->GetMatrix(matrix);
			igtl::PrintMatrix(matrix);
			std::cerr << "\n";
		}
	}
	else {
		std::cout << "Not Receiving TRANSFORM data type\n";
	}

}

int main() {
	std::signal(SIGINT, signal_handler);
	using namespace curan::communication;
	short port = 50000;
	asio::io_context io_context;
	auto lauchfunctor = [&io_context, port]() {
		foo(io_context, port);
	};
	std::jthread laucher(lauchfunctor);
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	Client client{ construction };
	auto connectionstatus = client.connect(bar);
	auto val = io_context.run();
	std::cout << "stopped running\n";
	return 0;
}