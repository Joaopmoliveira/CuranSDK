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
#include "processmanagement/SingleChildParent.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <thread>

static constexpr char SKSL[] =
"uniform float3 in_resolution;"
"uniform float  in_time;"
"float f(vec3 p) {"
"    p.z -= in_time * 10.;"
"    float a = p.z * .1;"
"    p.xy *= mat2(cos(a), sin(a), -sin(a), cos(a));"
"    return .1 - length(cos(p.xy) + sin(p.yz));"
"}"
"half4 main(vec2 fragcoord) { "
"    vec3 d = .5 - fragcoord.xy1 / in_resolution.y;"
"    vec3 p=vec3(0);"
"    for (int i = 0; i < 32; i++) {"
"      p += f(p) * d;"
"    }"
"    return ((sin(p) + vec3(2, 5, 12)) / length(p)).xyz1;"
"}";


asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal)
{
	if (ptr_ctx) ptr_ctx->stop();
}

int viewer_code(asio::io_context& io_context,curan::process::ProcessLaucher* parent) {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	    auto button1 = Button::make("Temporal Calibration",resources);
	    button1->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button1->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

	    auto button2 = Button::make("Spatial Calibration",resources);
	   	button2->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button2->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

	    auto button3 = Button::make("Volume ROI",resources);
	   	button3->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button3->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

		auto button4 = Button::make("Reconstruction",resources);
	   	button4->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button4->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

		auto button5 = Button::make("Registration",resources);
	   	button5->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button5->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

		auto button6 = Button::make("Neuro Navigation",resources);
	   	button6->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button6->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!parent->handles){
				parent->async_lauch_process([inbut](bool sucess) { if(sucess) inbut->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
			} else {
				parent->async_terminate(std::chrono::milliseconds(300),[inbut,parent](){ inbut->set_waiting_color(SK_ColorRED);});
			}
		});

		RuntimeEffect effects{SKSL};
	    auto widgetcontainer = Container::make(Container::ContainerType::VARIABLE_CONTAINER,Container::Arrangement::VERTICAL,effects);
                    
	    *widgetcontainer << std::move(button1) 
                        << std::move(button2) 
                        << std::move(button3) 
                        << std::move(button4) 
                        << std::move(button5) 
                        << std::move(button6);

		widgetcontainer->set_variable_layout({SkRect::MakeXYWH(0.0,0.0,0.3333,0.5),SkRect::MakeXYWH(0.3332,0.0,0.3333,0.5),SkRect::MakeXYWH(0.6665,0.0,0.3333,0.5),
											  SkRect::MakeXYWH(0.0,0.5,0.3333,0.5),SkRect::MakeXYWH(0.3332,0.5,0.3333,0.5),SkRect::MakeXYWH(0.6665,0.5,0.3333,0.5)});

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page};
		config_draw.stack_page->stack(Loader::make("human_robotics_logo.jpeg",resources));

		viewer->set_minimum_size(page.minimum_size());

		while (!glfwWindowShouldClose(viewer->window) && !ptr_ctx->stopped()) {
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
		auto parent = std::make_unique<curan::process::ProcessLaucher>(io_context, std::chrono::milliseconds(100),std::chrono::milliseconds(0), 10);

		std::thread th{[&](){ 
			viewer_code(io_context,parent.get());
		}
		};

		io_context.run();
		th.join();
	}
	catch (std::exception& e) {
		std::cout << "exception thrown : " << e.what() << std::endl;
		return 1;
	}
	catch (...) {
		std::cout << "exception thrown : " << std::endl;
		return 1;
	}
	return 0;
}