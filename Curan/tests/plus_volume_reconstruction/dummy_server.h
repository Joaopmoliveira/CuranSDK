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
		std::cout << "Stopping context\n";
		cxt.stop();
	}
	catch (std::exception& e) {
		std::cout << "CLient exception was thrown" << e.what() << std::endl;
	}
}