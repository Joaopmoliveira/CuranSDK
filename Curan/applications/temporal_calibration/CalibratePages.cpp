#include "CalibratePages.h"

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;
	//---------------------- row Threshold -------------------//
	auto slider = Slider::make({ 0.0, 300.0f});
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob = TextBlob::make("Aquisition Time (Up to 30s)");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val = (processing->aquisition_time);
	auto textblob2 = TextBlob::make(std::to_string(current_val));
	slider->set_current_value(current_val);
	slider->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->aquisition_time =  (slider->get_current_value())*30;
	});

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(textblob) << std::move(textblob2) << std::move(slider);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });
	/*
	auto slider1 = Slider::make({ 0.0f, 300.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob1 = TextBlob::make("Maximum Radius");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val1 = (processing->configuration.maximum_radius - processing->configuration.maximum_radius_limit[0]) / (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
	slider1->set_current_value(current_val1);
	slider1->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.maximum_radius = processing->configuration.maximum_radius_limit[0] + slider->get_current_value() * (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
	});

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(textblob1) << std::move(slider1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });
*/
	auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*slidercontainer << /*std::move(container) <<*/ std::move(container);

	return Overlay::make(std::move(slidercontainer),SK_ColorTRANSPARENT,true);
}

std::unique_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto button = Button::make("RANSAC",resources);
	button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(150, 80));
	button->add_press_call([&processing](Button* button, Press press ,ConfigDraw* config) {
			
	});

	auto button2 = Button::make("Segmentation",resources);
	button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(150, 80));
	button2->add_press_call([&processing,&resources](Button* button, Press press ,ConfigDraw* config) {

	});

	auto button3 = Button::make("Data aquisition",resources);
	button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(150, 80));
	button3->add_press_call([&processing,&resources](Button* button, Press press ,ConfigDraw* config) {
		config->stack_page->stack(create_filtercontroler_overlay(processing,resources));
	});


	auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*viwers_container << std::move(button) << std::move(button2) << std::move(button3);
	viwers_container->set_divisions({0.0 , 0.5 , 1.0});
	viwers_container->set_color(SK_ColorTRANSPARENT);
	
	return Overlay::make(std::move(viwers_container),SkColorSetARGB(10,125,125,125),true);
}

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

	ObservationEigenFormat observation_n;

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	start_connection->add_press_call(start_connection_callback);
	auto start_connection_pointer = start_connection.get();

	auto segmentation = Button::make("Segmentation Visualizer",resources);
	segmentation->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	segmentation->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
		//processing->show_line.load();
		auto val = !processing->show_line.load();
		processing->show_line.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	});

	auto lines = Button::make("Show Scan Lines",resources);
	lines->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	lines->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
		auto val = !processing->show_calibration_lines.load();
		processing->show_calibration_lines.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	});
	
	auto points = Button::make("Show Segmented Points",resources);
	points->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	points->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
		auto val = !processing->show_pointstofit.load();
		processing->show_pointstofit.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	});
	
	auto calibration = Button::make("Start calibration",resources);
	calibration->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	calibration->add_press_call([processing](Button* button, Press press ,ConfigDraw* config) {
		//processing->start_calibration.load();
		auto val = !processing->start_calibration.load();
		processing->start_calibration.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);;
		processing->list_of_recorded_points.clear();
		processing->projections.clear();
		processing->normalized_position_signal.clear();
		processing->normalized_video_signal.clear();
		processing->timer = 0;
	});

	auto button_options = Button::make("Options",resources);
	button_options->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_options->add_press_call([&processing,&resources](Button* button,Press press , ConfigDraw* config) {
		config->stack_page->stack(create_options_overlay(processing,resources));
	});

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(segmentation) << std::move(lines) << std::move(points) << std::move(calibration) << std::move(button_options);
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}