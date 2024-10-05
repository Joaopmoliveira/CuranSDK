#include "CalibratePages.h"
#include <cmath>
#include <chrono>
#include "userinterface/widgets/ImutableTextPanel.h"

/*
	std::atomic<double> timestep = 0.05;
	std::atomic<size_t> iterations = 5;
	std::atomic<double> conductance = 3.0;
	std::atomic<double> sigma = 2.0;
*/

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;
	//---------------------- row Minimum Radius -------------------//
	auto slider = Slider::make({ 0.0f, 300.0f });
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob = TextBlob::make("Timestep");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val = (processing->timestep - processing->limits_timestep[0]) / (processing->limits_timestep[1] - processing->limits_timestep[0]);
	slider->set_current_value(current_val);
	slider->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->timestep = processing->limits_timestep[0] + slider->get_current_value() * (processing->limits_timestep[1] - processing->limits_timestep[0]);
	});

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(textblob) << std::move(slider);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Maximum Radius -------------------//
	auto slider1 = Slider::make({ 0.0f, 300.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob1 = TextBlob::make("Iterations");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val1 = (processing->iterations - processing->limits_iterations[0]) / (processing->limits_iterations[1] - processing->limits_iterations[0]);
	slider1->set_current_value(current_val1);
	slider1->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->iterations = processing->limits_iterations[0] + slider->get_current_value() * (processing->limits_iterations[1] - processing->limits_iterations[0]);
	});

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(textblob1) << std::move(slider1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Sweep Angle -------------------//
	auto slider2 = Slider::make({ 0.0f, 300.0f });
	slider2->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob2 = TextBlob::make("Conductance");
	textblob2->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val2 = (processing->conductance - processing->limits_conductance[0]) / (processing->limits_conductance[1] - processing->limits_conductance[0]);
	slider2->set_current_value(current_val2);
	slider2->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->conductance = processing->limits_conductance[0] + slider->get_current_value() * (processing->limits_conductance[1] - processing->limits_conductance[0]);
	});

	auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container2 << std::move(textblob2) << std::move(slider2);
	container2->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Sigma Gradient -------------------//
	auto slider3 = Slider::make({ 0.0f, 300.0f });
	slider3->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob3 = TextBlob::make("Sigma Gradient");
	textblob3->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val3 = (processing->sigma - processing->limits_sigma[0]) / (processing->limits_sigma[1] - processing->limits_sigma[0]);
	slider3->set_current_value(current_val3);
	slider3->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->sigma = processing->limits_sigma[0] + slider->get_current_value() * (processing->limits_sigma[1] - processing->limits_sigma[0]);
	});

	auto container3 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container3 << std::move(textblob3) << std::move(slider3);
	container3->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- final -------------------//

	auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*slidercontainer << std::move(container) << std::move(container1) << std::move(container2) << std::move(container3);

	return Overlay::make(std::move(slidercontainer),SK_ColorTRANSPARENT,true);
}

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

	auto button_options = Button::make("Options",resources);
	button_options->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_options->add_press_call([&processing,&resources](Button* button,Press press , ConfigDraw* config) {
		config->stack_page->stack(create_options_overlay(processing,resources));
	});

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(continuous_recording) << std::move(snapshot) << std::move(pointcloud) << std::move(resetpointcloud) << std::move(button_options);
	
	processing->button->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}