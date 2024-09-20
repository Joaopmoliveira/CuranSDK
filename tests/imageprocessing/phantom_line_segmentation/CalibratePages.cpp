#include "CalibratePages.h"
#include <cmath>
#include <chrono>
#include "userinterface/widgets/ImutableTextPanel.h"



curan::ui::Page create_main_page(std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto image_display = ImageDisplay::make();
	auto image_display_pointer = image_display.get();

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(image_display);

	processing = std::make_shared<ProcessingMessage>(image_display_pointer);

	auto start_connection_callback = [processing](Button* button, Press press ,ConfigDraw* config) {
		if (!processing->connection_status.value()) {
			curan::utilities::Job val{"connection thread",[processing]() { processing->communicate();}};
			processing->shared_pool->submit(val);
		}
		else {
			processing->attempt_stop();
		}
	};

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	start_connection->add_press_call(start_connection_callback);
	auto start_connection_pointer = start_connection.get();

	auto pointcloud = Button::make("Compute Pointcloud",resources);
	pointcloud->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	pointcloud->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {    
    auto val = !processing->compute_poincloud.load();
    processing->compute_poincloud.store(val);
    processing->pointcloud_finished.store(false);

    });

	auto resetpointcloud = Button::make("Reset Pointcloud",resources);
	resetpointcloud->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	resetpointcloud->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {    
    processing->list_of_recorded_points.clear();
    });

	auto calibration = Button::make("Start pointcloud colection",resources);
	calibration->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	calibration->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
		//processing->start_calibration.load();
		auto val = !processing->start_calibration.load();
		processing->start_calibration.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);;
		//processing->list_of_recorded_points.clear();
		processing->projections.clear();
		processing->normalized_position_signal.clear();
		processing->normalized_video_signal.clear();
		processing->timer = 0;
		processing->calibration_value = 0;
    	processing->start_time = std::chrono::steady_clock::now();
        processing->shifted_signal.clear();
	});


/*
	auto button_results = Button::make("Results",resources);
	button_results->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_results->add_press_call([&processing,&resources](Button* button,Press press , ConfigDraw* config) {
		config->stack_page->stack(create_results_overlay(processing,resources));
	});
*/
	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(calibration) << std::move(pointcloud) << std::move(resetpointcloud);
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}