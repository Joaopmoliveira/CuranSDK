#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>

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

void foo(asio::io_context& cxt, unsigned short port) {
	using namespace curan::communication;
	try {
		interface_igtl igtlink_interface;
		Server::Info construction{ cxt,igtlink_interface ,port };

		Server server{ construction };

		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();
		int counter = 0;
		while (counter< 1000) {
			auto start = std::chrono::high_resolution_clock::now();
			igtl::Matrix4x4 matrix;
			GetRandomTestMatrix(matrix);
			ts->GetTime();

			igtl::TransformMessage::Pointer transMsg;
			transMsg = igtl::TransformMessage::New();
			transMsg->SetDeviceName("Tracker");

			transMsg->SetMatrix(matrix);
			transMsg->SetTimeStamp(ts);
			transMsg->Pack();

			auto callable = [transMsg]() {
				return asio::buffer(transMsg->GetPackPointer(), transMsg->GetPackSize());
			};
			auto to_send = curan::utilities::CaptureBuffer::make_shared(std::move(callable));
			server.write(to_send);
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(10) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
			++counter;
		}
		curan::utilities::cout << "Stopping context";
		cxt.stop();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "CLient exception was thrown" + std::string(e.what());
	}
}

void bar(size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	curan::utilities::cout << "received message";
	assert(val.IsNotNull());
	if (!er) {
		std::string tmp = val->GetMessageType();
		std::string desired = "TRANSFORM";
		if (!tmp.compare(desired)) {
			curan::utilities::cout << "Receiving TRANSFORM data type";
			igtl::TransformMessage::Pointer transMsg = igtl::TransformMessage::New();
			//transMsg->Copy(val);
			//int c = transMsg->Unpack(1);

			//if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
			//{
			//	// Retrive the transform data
			//	igtl::Matrix4x4 matrix;
			//	transMsg->GetMatrix(matrix);
			//}
		}
		else {
			curan::utilities::cout << "Not Receiving TRANSFORM data type";
		}
	}
	else {
		curan::utilities::cout << "failed";
	}

}

int main() {
	try {
		curan::utilities::cout << "started running";
		using namespace curan::communication;
		unsigned short port = 50000;
		asio::io_context io_context;
		auto lauchfunctor = [&io_context, port]() {
			foo(io_context, port);
		};
		std::thread laucher(lauchfunctor);
		interface_igtl igtlink_interface;
		Client::Info construction{ io_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		Client client{ construction };
		auto connectionstatus = client.connect(bar);
		auto val = io_context.run();
		curan::utilities::cout << "stopped running";
		laucher.join();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "CLient exception was thrown"+std::string(e.what());
		return 1;
	}
	return 0;
}