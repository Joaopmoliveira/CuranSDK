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
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>

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
			unsigned char val = (int)((intensity + 0.5) * 255);
			image.set(c, r, val);
		}
	}
	return image;
}

void generate_image_message(std::shared_ptr<curan::ui::OpenIGTLinkViewer> button, std::shared_ptr<curan::ui::ImageDisplay> pure_display) {
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
	while (std::chrono::duration<float, std::chrono::seconds::period>(start - genesis).count() < 20.0) {
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

		auto lam = [message_to_receive, imgMsg, width = img.width(), height = img.height()](SkPixmap& requested) {
			auto inf = SkImageInfo::Make(width, height, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
			size_t row_size = width * sizeof(char);
			SkPixmap map{ inf,imgMsg->GetScalarPointer(),row_size };
			requested = map;
			return;
		};

		pure_display->update_image(lam);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		++counter;

	}
	curan::utils::cout << "stopped to send data";

}

std::shared_ptr<curan::ui::Page> create_option_page() {
	using namespace curan::ui;
	IconResources resources{ "C:/dev/Curan/resources" };

	SkColor colbuton = { SK_ColorBLACK };
	SkColor coltext = { SK_ColorBLACK };

	SkPaint paint_square;
	paint_square.setStyle(SkPaint::kFill_Style);
	paint_square.setAntiAlias(true);
	paint_square.setStrokeWidth(4);
	paint_square.setColor(colbuton);

	SkPaint paint_text;
	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(coltext);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SK_ColorTRANSPARENT);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

	SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	Button::Info infor{ resources };
	infor.button_text = "Slide";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorCYAN;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeWH(100, 80);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);
	infor.button_text = "Push";
	std::shared_ptr<Button> button2 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.paint_layout = paint_square2;
	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.5 , 1.0 };
	info.layouts = { button,button2 };
	std::shared_ptr<Container> container = Container::make(info);

	Page::Info information;
	information.backgroundcolor = SK_ColorTRANSPARENT;
	information.contained = container;
	return Page::make(information);
}


int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorBLACK };
		SkColor coltext = { SK_ColorWHITE };

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

		SkPaint paint_text;
		paint_text.setStyle(SkPaint::kFill_Style);
		paint_text.setAntiAlias(true);
		paint_text.setStrokeWidth(4);
		paint_text.setColor(coltext);

		const char* fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SK_ColorBLACK);

		OpenIGTLinkViewer::Info infoviewer;
		infoviewer.text_font = text_font;
		infoviewer.size = SkRect::MakeWH(600, 500);
		std::shared_ptr<OpenIGTLinkViewer> open_viwer = OpenIGTLinkViewer::make(infoviewer);

		ImageDisplay::Info processed_viwer_info;
		processed_viwer_info.height = 0;
		processed_viwer_info.width = 0;
		std::shared_ptr<ImageDisplay> processed_viwer = ImageDisplay::make(processed_viwer_info);

		Container::InfoLinearContainer info;
		info.paint_layout = paint_square2;
		info.arrangement = curan::ui::Arrangement::HORIZONTAL;
		info.divisions = { 0.0 , 0.5 , 1.0 };
		info.layouts = { open_viwer,processed_viwer };
		std::shared_ptr<Container> container2 = Container::make(info);

		Button::Info infor{ resources };
		infor.button_text = "Connect";
		infor.click_color = SK_ColorGRAY;
		infor.hover_color = SK_ColorDKGRAY;
		infor.waiting_color = SK_ColorBLACK;
		infor.icon_identifier = "";
		infor.paintButton = paint_square;
		infor.paintText = paint_text;
		infor.size = SkRect::MakeWH(100, 80);
		infor.textFont = text_font;
		std::shared_ptr<Button> button = Button::make(infor);

		infor.button_text = "Options";
		ConfigDraw config_draw;

		auto page_option = create_option_page();

		infor.callback = [&config_draw, &page_option]() {
			config_draw.stack_page.push(page_option);
		};
		std::shared_ptr<Button> buttonoptions = Button::make(infor);

		info.arrangement = curan::ui::Arrangement::HORIZONTAL;
		info.divisions = { 0.0 , 0.5 , 1.0 };
		info.layouts = { button,buttonoptions };
		std::shared_ptr<Container> buttoncontainer = Container::make(info);

		info.arrangement = curan::ui::Arrangement::VERTICAL;
		info.divisions = { 0.0 , 0.1 , 1.0 };
		info.layouts = { buttoncontainer,container2 };
		std::shared_ptr<Container> container = Container::make(info);

		auto rec = viewer->get_size();
		Page::Info information;
		information.backgroundcolor = SK_ColorBLACK;
		information.contained = container;
		std::shared_ptr<Page> page = Page::make(information);
		page->propagate_size_change(rec);
		page_option->propagate_size_change(rec);
		int width = rec.width();
		int height = rec.height();

		auto lamd = [open_viwer, processed_viwer]() {
			generate_image_message(open_viwer, processed_viwer);
		};
		std::thread message_generator{ lamd };

		auto blur = SkImageFilters::Blur(10, 10, nullptr);
		SkPaint bluring_paint;
		bluring_paint.setImageFilter(std::move(blur));
		SkSamplingOptions options;
		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			auto temp_height = pointer_to_surface->height();
			auto temp_width = pointer_to_surface->width();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (temp_height != height || temp_width != width) {
				rec = SkRect::MakeWH(temp_width, temp_height);
				page->propagate_size_change(rec);
				page_option->propagate_size_change(rec);
			}
			page->draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!config_draw.stack_page.empty()) {
				auto image = pointer_to_surface->makeImageSnapshot();
				canvas->drawImage(image, 0, 0, options, &bluring_paint);
				config_draw.stack_page.top()->draw(canvas);

				if (!signals.empty() && !config_draw.stack_page.top()->propagate_signal(signals.back(), &config_draw)) {
					std::visit(curan::utils::overloaded{
						[](Empty arg) {

							},
						[](Move arg) {

							},
						[&config_draw](Press arg) {
								config_draw.stack_page.pop();
							},
						[](Scroll arg) {;

							},
						[](Unpress arg) {

							},
						[](Key arg) {

							},
						[](ItemDropped arg) {;

						} },
						signals.back());
				}
			}
			else if(!signals.empty()){
				page->propagate_signal(signals.back(), &config_draw);
			}

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::cout << "delay: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		message_generator.join();
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}