#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"
#include "utils/Flag.h"
#include "utils/TheadPool.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include <cmath>
#include <csignal>
#include <system_error>

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
#include "userinterface/widgets/Loader.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>

asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal){	
	if (ptr_ctx) ptr_ctx->stop();
}

class ChildProcess {
	asio::io_context& hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Client> client;
	bool first_connection_established = false;
	size_t numbers_of_triggered_connections = 0;
public:
	template <class _Rep, class _Period>
	ChildProcess(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_violations, unsigned short port = 50000) :
		hidden_context{ client_ctx },
		timer{ hidden_context },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_violations }
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		curan::communication::interface_prochandler igtlink_interface;
		curan::communication::Client::Info construction{ hidden_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(hidden_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		client = curan::communication::Client::make(construction,
			std::chrono::seconds(10),
			[this](std::error_code er) {
				if (!er) {
					first_connection_established = true;
				}
				else {
					client->get_socket().close();
					timer.cancel();
					
				}
			}
		);
		client->connect([this](const size_t& prot,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(prot, er, val);
			});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (first_connection_established) number_of_violations = was_violated ? number_of_violations + 1 : 0;
		
		was_violated = true;
		if (number_of_violations > max_num_violations) {
			timer.cancel();
			client->get_socket().close();
			return;
		}
		if(numbers_of_triggered_connections<10){
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
			val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
			if (first_connection_established) {
				client->write(to_send);
			}
		} else {

		}

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			timer.cancel();
			client->get_socket().close();
			return;
		}
		switch (val->signal_to_process) {
		case curan::communication::ProcessHandler::Signals::HEART_BEAT:
			was_violated = false;
			number_of_violations = 0;
			break;
		case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
		default:
			timer.cancel();
			client->get_socket().close();
			hidden_context.get_executor().on_work_finished();
			break;
		}
	}

	~ChildProcess() {
		timer.cancel();
		client->get_socket().close();
	}
};


int viewer_code() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	    auto button1 = Button::make("Temporal Calibration",resources);
	    button1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(300, 300));

		auto icon = resources.get_icon("hr_repeating.png");
	    auto widgetcontainer =  Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	    *widgetcontainer << std::move(button1);

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page};
		config_draw.stack_page->stack(Loader::make("human_robotics_logo.jpeg",resources));

		viewer->set_minimum_size(page.minimum_size());

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
			page.propagate_heartbeat(&config_draw);
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}

int main() {
	try {
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;
		std::unique_ptr<ChildProcess> child;
		
		child = std::make_unique<ChildProcess>(io_context, std::chrono::milliseconds(100), 10);
		io_context.run();
	}
	catch (std::exception& e) {
		return 1;
	}
	catch (...) {
		return 1;
	}
	return 0;
}