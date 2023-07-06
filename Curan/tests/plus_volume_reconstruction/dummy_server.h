#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <iostream>
#include <optional>
#include <map>


class ImageTesting {
	int _width;
	int _height;
	std::unique_ptr<unsigned char[]> buffer;
public:

	ImageTesting(int width, int height) : _width{ width }, _height{ height } {
		buffer = std::unique_ptr<unsigned char[]>(new unsigned char[width * height]);
	}

	inline int width() {
		return _width;
	}

	inline int height() {
		return _height;
	}

	inline void set(int w, int h, char val) {
		unsigned char* loc = nullptr;
		if (buffer) {
			loc = buffer.get();
			loc[w+h*height()] = val;
		}
	}

	inline int size() {
		return _width * _height;
	}

	unsigned char* get_scalar_pointer() {
		if (buffer)
			return buffer.get();
		return nullptr;
	}
};

struct vec2 {
	float x;
	float y;

	vec2(float x, float y) : x{x}, y{y} {
	
	}

	float norm() {
		return std::sqrt(x*x+y*y);
	}

};

ImageTesting update_texture(ImageTesting image, float value){

	for (size_t r = 0; r < image.height(); ++r)
	{
		float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
		for (size_t c = 0; c < image.width(); ++c)
		{
			float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

			vec2 delta{ (r_ratio - 0.5f), (c_ratio - 0.5f) };

			float angle = std::atan2(delta.x, delta.y);

			float distance_from_center = delta.norm();

			float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;
			unsigned char val = (int)((intensity+0.5)*255);
			image.set(c, r, val);
		}
	}
	return image;
}

void foo(unsigned short port,asio::io_context& cxt,std::atomic<bool>& server_running) {
	using namespace curan::communication;
	try {
        std::cout << "Server starting\n";
		interface_igtl igtlink_interface;
		Server::Info construction{ cxt,igtlink_interface ,port };

		Server server{ construction };

		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();
        ImageTesting img{100,100};

	    int   size[] = { img.width(), img.height(), 1}; 
	    float spacing[] = { 1.0, 1.0, 5.0 };  
	    int   svsize[] = { img.width(), img.height(), 1};   
	    int   svoffset[] = { 0, 0, 0 };  
	    int   scalarType = igtl::ImageMessage::TYPE_UINT8;

		int counter = 0;
        auto genesis = std::chrono::high_resolution_clock::now();

		while (server_running) {
			auto start = std::chrono::high_resolution_clock::now();
            float time = std::chrono::duration<float, std::chrono::seconds::period>(start - genesis).count();

			igtl::Matrix4x4 matrix;
			igtl::IdentityMatrix(matrix);

  			// random orientation
  			static float theta = 0.0;
			float orientation[4];
 			orientation[0]=0.0;
  			orientation[1]=0.6666666666*cos(time);
  			orientation[2]=0.577350269189626;
  			orientation[3]=0.6666666666*sin(time);

			igtl::QuaternionToMatrix(orientation,matrix);

			matrix[0][3] = 0.5*std::sin(time);
			matrix[1][3] = 0.0;
			matrix[2][3] = 0.0;

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

		    img = update_texture(std::move(img), 1.0+time);
		    ts->GetTime();

            igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
		    imgMsg->SetDimensions(size);
		    imgMsg->SetSpacing(spacing);
		    imgMsg->SetScalarType(scalarType);
		    imgMsg->SetDeviceName("ImagerClient");
		    imgMsg->SetSubVolume(svsize, svoffset);
		    imgMsg->AllocateScalars();

		    std::memcpy(imgMsg->GetScalarPointer(), img.get_scalar_pointer(), img.size());

		    imgMsg->SetMatrix(matrix);
		    imgMsg->Pack();

            auto callable2 = [imgMsg]() {
				return asio::buffer(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
			};
            auto to_send_image = curan::utilities::CaptureBuffer::make_shared(std::move(callable2));
			server.write(to_send_image);

			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16));
		}
		std::cout << "Stopping server\n";
	}
	catch (std::exception& e) {
		std::cout << "CLient exception was thrown" << e.what() << std::endl;
	}
}