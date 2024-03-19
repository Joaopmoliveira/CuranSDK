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

std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning, curan::ui::IconResources& resources){
        using namespace curan::ui;
        auto warn = Button::make(" ", "warning.png", resources);
        warn->set_click_color(SK_AlphaTRANSPARENT)
            .set_hover_color(SK_AlphaTRANSPARENT)
            .set_waiting_color(SK_AlphaTRANSPARENT)
            .set_size(SkRect::MakeWH(400, 200));

        auto button = Button::make(warning, resources);
        button->set_click_color(SK_AlphaTRANSPARENT)
            .set_hover_color(SK_AlphaTRANSPARENT)
            .set_waiting_color(SK_AlphaTRANSPARENT)
            .set_size(SkRect::MakeWH(200, 50));

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        *viwers_container << std::move(warn) << std::move(button);
        viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0, .8, 1.0});

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

int main() {
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
			if(config) config->stack_page->stack(warning_overlay("Started Temporal Calibration",resources));
		});

	    auto button2 = Button::make("Spatial Calibration",resources);
	   	button2->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button2->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(config) config->stack_page->stack(warning_overlay("Started Spatial Calibration",resources));
		});

	    auto button3 = Button::make("Volume ROI",resources);
	   	button3->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button3->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(config) config->stack_page->stack(warning_overlay("Started desired Volume Specification",resources));
		});

		auto button4 = Button::make("Reconstruction",resources);
	   	button4->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button4->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(config) config->stack_page->stack(warning_overlay("Started Volumetric Reconstruction",resources));
		});

		auto button5 = Button::make("Registration",resources);
	   	button5->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button5->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(config) config->stack_page->stack(warning_overlay("Started Real-Time Registration",resources));
		});

		auto button6 = Button::make("Neuro Navigation",resources);
	   	button6->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(SK_ColorGRAY)
                .set_size(SkRect::MakeWH(300, 300));
		button6->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(config) config->stack_page->stack(warning_overlay("Started Neuro-Navigation",resources));
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