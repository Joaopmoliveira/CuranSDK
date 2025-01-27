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
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include <atomic>
#include "imageprocessing/VolumeReconstructor.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;

using OutputPixelType = float;
using InputImageType = itk::Image<unsigned char, 3>;
using OutputImageType = itk::Image<OutputPixelType, 3>;
using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;

struct SharedState{
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> texture;
    curan::renderable::Window & window;
    asio::io_context& context;
	std::vector<igtl::ImageMessage::Pointer> recorded_images;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context) : window{in_window},context{in_context} {}
};

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

void foo(unsigned short port,asio::io_context& cxt) {
	using namespace curan::communication;
	try {
		auto server = Server<protocols::igtlink>::make(cxt,port);

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

		double controled_time = 0.0;

		while (!cxt.stopped()) {
			auto start = std::chrono::high_resolution_clock::now();
            float time = std::chrono::duration<float, std::chrono::seconds::period>(start - genesis).count();

			igtl::Matrix4x4 matrix;
			igtl::IdentityMatrix(matrix);

  			// random orientation
			float orientation[4];
 			orientation[0]=0.0;
  			orientation[1]=0.6666666666*cos(time);
  			orientation[2]=0.577350269189626;
  			orientation[3]=0.6666666666*sin(time);

			//igtl::QuaternionToMatrix(orientation,matrix);

			matrix[0][3] = 0.0;
			matrix[1][3] = 0.25*std::sin(10.0*controled_time);
			matrix[2][3] = 0.0;

			ts->GetTime();

			igtl::TransformMessage::Pointer transMsg;
			transMsg = igtl::TransformMessage::New();
			transMsg->SetDeviceName("Tracker");

			transMsg->SetMatrix(matrix);
			transMsg->SetTimeStamp(ts);
			transMsg->Pack();

			auto to_send = curan::utilities::CaptureBuffer::make_shared(transMsg->GetPackPointer(), transMsg->GetPackSize(),transMsg);
			server->write(to_send);

		    img = update_texture(std::move(img), 1.0+controled_time);
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

            auto to_send_image = curan::utilities::CaptureBuffer::make_shared(imgMsg->GetPackPointer(), imgMsg->GetPackSize(),imgMsg);
			server->write(to_send_image);

			std::this_thread::sleep_for(std::chrono::milliseconds(30));

			controled_time += 0.01;
		}
		std::cout << "Stopping server\n";
	}
	catch (std::exception& e) {
		std::cout << "CLient exception was thrown" << e.what() << std::endl;
	}
}