#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/SingletonIconResources.h"
#include "userinterface/widgets/Page.h"
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/Job.h"
#include "utils/TheadPool.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <iostream>

struct ProcessingMessage {

	std::shared_ptr<curan::ui::ImageDisplay> processed_viwer;
	std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer;
	std::shared_ptr<curan::utils::Flag> connection_status;
	std::shared_ptr<curan::ui::Button> button;
	curan::communication::Client* client_pointer = nullptr;
	short port = 10000;

	ProcessingMessage(std::shared_ptr<curan::ui::ImageDisplay> in_processed_viwer, 
					  std::shared_ptr<curan::ui::OpenIGTLinkViewer> in_open_viwer, 
					  std::shared_ptr<curan::utils::Flag> flag) : connection_status{ flag }, processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer } 
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		if (!er) {
			assert(val.IsNotNull());
			std::string tmp = val->GetMessageType();
			std::string transform = "TRANSFORM";
			std::string image = "IMAGE";
			if (!tmp.compare(transform)) {
				open_viwer->process_message(val);
			}
			else if (!tmp.compare(image)) {
				open_viwer->process_message(val);
				igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
				message_body->Copy(val);
				int c = message_body->Unpack(1);
				if (c & igtl::MessageHeader::UNPACK_BODY) {
					auto lam = [message_body](SkPixmap& requested) {
						int x, y, z;
						message_body->GetDimensions(x,y,z);
						auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
						size_t row_size = x * sizeof(char);
						SkPixmap map{ inf,message_body->GetScalarPointer(),row_size };
						requested = map;
						return;
					};
					processed_viwer->update_image(lam);
				}
			}
			else {
				std::cout << "Unknown Message\n";
			}
			return false;
		}
		else {
			return true;
		}
	};

	void communicate() {
		
		using namespace curan::communication;
		button->update_color(SK_ColorGREEN);
		asio::io_context io_context;
		interface_igtl igtlink_interface;
		Client::Info construction{ io_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		Client client{ construction };
		connection_status->set();
		client_pointer = &client;

		auto lam = [this, &io_context](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
			if (process_message(protocol_defined_val,er,val))
			{
				connection_status->clear();
				io_context.stop();

			}
		};
		auto connectionstatus = client.connect(lam);
		auto val = io_context.run();
		button->update_color(SK_ColorRED);
		client_pointer = nullptr;

		return;
	}

	void attempt_stop() {
		if (client_pointer)
			client_pointer->get_socket().close();
	}
};



int main(int argc, char* argv[]) {
	//initualize the thread pool;
	curan::utils::initialize_thread_pool(10);
	if (argc != 2) {
		std::cout << "the ultrasound calibration app only parses one argument, the port of the server to connect to\n";
		return 1;
	}

	std::string val = { argv[1] };
	size_t pos = 0;
	int port = 0;
	try{
		port = std::stoi(val, &pos);
	} catch (...) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 2;
	}

	std::cout << "the received port is: " << port << "\n";

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

	SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SK_ColorBLACK);

	OpenIGTLinkViewer::Info infoviewer;
	infoviewer.text_font = text_font;
	infoviewer.size = SkRect::MakeWH(0, 0);
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

	auto flag = curan::utils::Flag::make_shared_flag();

	std::shared_ptr<ProcessingMessage> processing = std::make_shared<ProcessingMessage>(processed_viwer, open_viwer,flag);
	processing->port = port;

	auto lam = [processing]() {
		if (!processing->connection_status->value()) {
			curan::utils::Job val;
			val.description = "connection thread";
			val.function_to_execute = [processing]() {
				processing->communicate();
			};
			curan::utils::pool->submit(val);
		}
		else {
			processing->attempt_stop();
		}
	};

	Button::Info infor;
	infor.button_text = "Connect";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorRED;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeWH(100, 80);
	infor.textFont = text_font;
	infor.callback = lam;
	std::shared_ptr<Button> button = Button::make(infor);

	processing->button = button;
	button->update_color(SK_ColorRED);

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
	processing->attempt_stop();
	curan::utils::terminate_thread_pool();
	return 0;
}