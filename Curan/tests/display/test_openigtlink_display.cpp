#include "userinterface/Window.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include <csignal>
#include "userinterface/widgets/ConfigDraw.h"
#include "utils/Logger.h"

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

void generate_image_message(curan::ui::OpenIGTLinkViewer* button) {
	ImageTesting img{100,100};

	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();

	int   size[] = { img.width(), img.height(), 1}; 
	float spacing[] = { 1.0, 1.0, 5.0 };  
	int   svsize[] = { img.width(), img.height(), 1};   
	int   svoffset[] = { 0, 0, 0 };  
	int   scalarType = igtl::ImageMessage::TYPE_UINT8;

	size_t counter = 0;
	auto genesis = std::chrono::high_resolution_clock::now();
	while (counter < 10) {
		auto start = std::chrono::high_resolution_clock::now();
		float time = std::chrono::duration<float, std::chrono::seconds::period>(start - genesis).count();
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

		igtl::Matrix4x4 matrix;
		GetRandomTestMatrix(matrix);
		imgMsg->SetMatrix(matrix);
		imgMsg->Pack();

		igtl::MessageHeader::Pointer header_to_receive = igtl::MessageHeader::New();
		header_to_receive->InitPack();
		std::memcpy(header_to_receive->GetPackPointer(), imgMsg->GetPackPointer(), header_to_receive->GetPackSize());

		header_to_receive->Unpack();
		igtl::MessageBase::Pointer message_to_receive = igtl::MessageBase::New();
		message_to_receive->SetMessageHeader(header_to_receive);
		message_to_receive->AllocatePack();
		std::memcpy(message_to_receive->GetPackBodyPointer(), imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());

		button->process_message(message_to_receive);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(500) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		++counter;
	}
	curan::utilities::cout << "stopped to send data";

}

void generate_transform_message(curan::ui::OpenIGTLinkViewer* button) {
	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();
	size_t counter = 0;
	while(counter<10){
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

		igtl::MessageHeader::Pointer header_to_receive = igtl::MessageHeader::New();
		header_to_receive->InitPack();
		transMsg->GetBufferPointer();
		std::memcpy(header_to_receive->GetPackPointer(), transMsg->GetPackPointer(), header_to_receive->GetPackSize());

		header_to_receive->Unpack();
		igtl::MessageBase::Pointer message_to_receive = igtl::MessageBase::New();
		message_to_receive->SetMessageHeader(header_to_receive);
		message_to_receive->AllocatePack();
		std::memcpy(message_to_receive->GetPackBodyPointer(), transMsg->GetPackBodyPointer(),transMsg->GetPackBodySize());

		button->process_message(message_to_receive);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(500) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		++counter;
	}
	curan::utilities::cout << "stopped to send data";
}


int main() {
	try {
		using namespace curan::ui;
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto igtlink_viewer = OpenIGTLinkViewer::make();
		igtlink_viewer->set_size(SkRect::MakeWH(600,600));
		igtlink_viewer->compile();

		auto caldraw = igtlink_viewer->draw();
		auto calsignal = igtlink_viewer->call();

		SkRect rect = SkRect::MakeLTRB(0, 0, 1200, 800);
		igtlink_viewer->set_position(rect);

		auto pointer_to_igtlink_viewer = igtlink_viewer.get();
		auto lamd = [pointer_to_igtlink_viewer]() {
			generate_image_message(pointer_to_igtlink_viewer);
		};
		std::thread message_generator{ lamd };

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);
			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				calsignal(signals.back(),&config_draw);
			bool val = viewer->swapBuffers();
			if (!val)
				curan::utilities::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		curan::utilities::cout << "stopped window";
		message_generator.join();
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}