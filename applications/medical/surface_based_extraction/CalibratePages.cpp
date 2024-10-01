#include "CalibratePages.h"
#include <cmath>
#include <chrono>
#include "userinterface/widgets/ImutableTextPanel.h"



curan::ui::Page create_main_page(std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto image_display = ImageDisplay::make();
	auto filtered_image_display = ImageDisplay::make();

	processing = std::make_shared<ProcessingMessage>(image_display.get(),filtered_image_display.get());

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(image_display) << std::move(filtered_image_display);



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
	processing->button = start_connection.get();

	auto continuous_recording = Button::make("Continuous Recording",resources);
	continuous_recording->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	continuous_recording->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
    	processing->snapshot = false;
		processing->store_to_file = false;
		processing->record_poincloud = !processing->record_poincloud;
		if(processing->record_poincloud)
			button->set_waiting_color(SK_ColorCYAN);
		else
			button->set_waiting_color(SK_ColorBLACK);
	});
	processing->button_start_collection = continuous_recording.get();

	auto snapshot = Button::make("Snapshot Recording",resources);
	snapshot->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	snapshot->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {  
		processing->store_to_file = false;
		processing->record_poincloud = false;  
    	processing->snapshot = true;
		processing->button_start_collection->set_waiting_color(SK_ColorBLACK);
    });

	auto pointcloud = Button::make("Save Pointcloud",resources);
	pointcloud->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	pointcloud->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {    
    	processing->snapshot = false;
		processing->record_poincloud = false;
		processing->store_to_file = true;
		processing->button_start_collection->set_waiting_color(SK_ColorBLACK);
    });


	auto resetpointcloud = Button::make("Clear Pointcloud",resources);
	resetpointcloud->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	resetpointcloud->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {    
    	processing->snapshot = false;
		processing->record_poincloud = false;
		processing->store_to_file = false;
		processing->button_start_collection->set_waiting_color(SK_ColorBLACK);
    	processing->list_of_recorded_points.clear();
    });

/*
	auto button_results = Button::make("Results",resources);
	button_results->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_results->add_press_call([&processing,&resources](Button* button,Press press , ConfigDraw* config) {
		config->stack_page->stack(create_results_overlay(processing,resources));
	});
*/
	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(continuous_recording) << std::move(snapshot) << std::move(pointcloud) << std::move(resetpointcloud);
	

	processing->button->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}