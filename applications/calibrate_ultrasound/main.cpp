#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/SingletonIconResources.h"
#include "userinterface/widgets/Page.h"
#include "utils/Logger.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <iostream>

void message_callback(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	curan::utils::cout << "received message";
	assert(val.IsNotNull());
	std::string tmp = val->GetMessageType();
	std::string transform = "TRANSFORM";
	std::string image = "IMAGE";
	if (!er) {
		if (!tmp.compare(transform)) {
			std::cout << "Receiving TRANSFORM data type\n";
		}
		else if (!tmp.compare(image)) {
			std::cout << "Not Receiving TRANSFORM data type\n";
		}
		else {
			std::cout << "Unknown Message\n";
		}
	}
}

struct ProcessingMessage {
	std::shared_ptr<curan::ui::ImageDisplay> processed_viwer;
	std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer;
	bool should_contiue;

	ProcessingMessage(std::shared_ptr<curan::ui::ImageDisplay> in_processed_viwer, std::shared_ptr<curan::ui::OpenIGTLinkViewer> in_open_viwer) : processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer } {}

	void process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		curan::utils::cout << "received message";
		assert(val.IsNotNull());
		std::string tmp = val->GetMessageType();
		std::string transform = "TRANSFORM";
		std::string image = "IMAGE";
		if (!er) {
			if (!tmp.compare(transform)) {
				std::cout << "Receiving TRANSFORM data type\n";
			}
			else if (!tmp.compare(image)) {
				std::cout << "Not Receiving TRANSFORM data type\n";
			}
			else {
				std::cout << "Unknown Message\n";
			}
		}
	};

	bool continue_running() {
	
	}
};

int communication_proc(std::shared_ptr<curan::ui::ImageDisplay> processed_viwer, std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer) {
	using namespace curan::communication;
	short port = 50000;
	asio::io_context io_context;
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	Client client{ construction };
	auto connectionstatus = client.connect(message_callback);
	auto val = io_context.run();
}

int main() {
	try {
		using namespace curan::ui;

		IconResources* resources = IconResources::Load("C:/dev/Curan/resources");
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

		SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
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

		Button::Info infor;
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

		info.arrangement = curan::ui::Arrangement::VERTICAL;
		info.divisions = { 0.0 , 0.1 , 1.0 };
		info.layouts = { button,container2 };
		std::shared_ptr<Container> container = Container::make(info);

		auto rec = viewer->get_size();
		Page::Info information;
		information.backgroundcolor = SK_ColorBLACK;
		information.contained = container;
		std::shared_ptr<Page> page = Page::make(information);
		page->propagate_size_change(rec);

		int width = rec.width();
		int height = rec.height();

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			auto temp_height = pointer_to_surface->height();
			auto temp_width = pointer_to_surface->width();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (temp_height != height || temp_width != width) {
				rec = SkRect::MakeWH(temp_width, temp_height);
				page->propagate_size_change(rec);
			}
			page->draw(canvas);
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				page->propagate_signal(signals.back());
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}