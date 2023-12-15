#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>

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
			loc[w + h * height()] = val;
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

	vec2(float x, float y) : x{ x }, y{ y } {

	}

	float norm() {
		return std::sqrt(x * x + y * y);
	}

};

ImageTesting update_texture(ImageTesting image, float value) {

	for (int32_t r = 0; r < image.height(); ++r)
	{
		float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
		for (int c = 0; c < image.width(); ++c)
		{
			float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

			vec2 delta{ (r_ratio - 0.5f), (c_ratio - 0.5f) };

			float angle = std::atan2(delta.x, delta.y);

			float distance_from_center = delta.norm();

			float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;
			unsigned char val = (int)((intensity + 0.5) * 255);
			image.set(c, r, val);
		}
	}
	return image;
}

void generate_image_message(std::atomic<bool>& continue_running,curan::ui::OpenIGTLinkViewer* button, curan::ui::ImageDisplay* pure_display) {
	ImageTesting img{ 100,100 };

	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();

	int   size[] = { img.width(), img.height(), 1 };
	float spacing[] = { 1.0, 1.0, 5.0 };
	int   svsize[] = { img.width(), img.height(), 1 };
	int   svoffset[] = { 0, 0, 0 };
	int   scalarType = igtl::ImageMessage::TYPE_UINT8;

	size_t counter = 0;
	auto genesis = std::chrono::high_resolution_clock::now();
	auto start = std::chrono::high_resolution_clock::now();
	while (continue_running.load()) {
		start = std::chrono::high_resolution_clock::now();
		float time = std::chrono::duration<float, std::chrono::seconds::period>(start - genesis).count();

		img = update_texture(std::move(img), 1.0 + time);
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

    	auto buff = curan::utilities::CaptureBuffer::make_shared(imgMsg->GetScalarPointer(),imgMsg->GetImageSize(),imgMsg);
    	curan::ui::ImageWrapper wrapper{buff,(size_t)img.width(),(size_t)img.height()};
		pure_display->update_image(wrapper);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		++counter;

	}
	curan::utilities::cout << "stopped to send data";

}

std::unique_ptr<curan::ui::Overlay> create_option_page(curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto button = Button::make("Slide",resources);
	button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorCYAN).set_size(SkRect::MakeWH(100, 80));

	auto button2 = Button::make("Push",resources);
	button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorCYAN).set_size(SkRect::MakeWH(100, 80));

	auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*container2 << std::move(button) << std::move(button2);
	container2->set_divisions({0.0 , 0.5 , 1.0});
	container2->set_color(SK_ColorTRANSPARENT);

	return Overlay::make(std::move(container2),SK_ColorTRANSPARENT,true);
}


int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto igtlink_viewer = OpenIGTLinkViewer::make();
		igtlink_viewer->set_size(SkRect::MakeWH(600,500));
		auto igtlink_viewer_pointer = igtlink_viewer.get();

		auto image_display = ImageDisplay::make();
		auto image_display_pointer = image_display.get();

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*container << std::move(igtlink_viewer) << std::move(image_display);
		container->set_divisions({ 0.0 , 0.5 , 1.0 });

		auto button = Button::make("Connect",resources);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

		auto buttonoptions = Button::make("Options",resources);
		buttonoptions->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

		auto button_callback = [&resources](Button* button,Press press,ConfigDraw* config) {
			if(config->stack_page!=nullptr)
				config->stack_page->stack(create_option_page(resources));
		};
		buttonoptions->add_press_call(button_callback);

		auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*buttoncontainer << std::move(button) << std::move(buttonoptions);
		buttoncontainer->set_divisions({ 0.0 , 0.5 , 1.0 });

		auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
		*widgetcontainer << std::move(buttoncontainer) << std::move(container);
		widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		std::atomic<bool> continue_running = true;

		auto lamd = [igtlink_viewer_pointer, image_display_pointer,&continue_running]() {
			generate_image_message(continue_running,igtlink_viewer_pointer, image_display_pointer);
		};
		std::thread message_generator{ lamd };

		ConfigDraw config_draw{ &page};

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();

			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated()) {
		    	page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				page.propagate_signal(signals.back(), &config_draw);

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		continue_running.store(false);
		message_generator.join();
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}