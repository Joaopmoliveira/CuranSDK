#include "CalibratePages.h"

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;
	//---------------------- row Minimum Radius -------------------//
	auto slider = Slider::make({ 0.0f, 300.0f });
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob = TextBlob::make("Minimum Radius");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val = (processing->configuration.minimum_radius - processing->configuration.minimum_radius_limit[0]) / (processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
	slider->set_current_value(current_val);
	slider->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.minimum_radius = processing->configuration.minimum_radius_limit[0] + slider->get_current_value() * (processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
	});

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(textblob) << std::move(slider);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Maximum Radius -------------------//
	auto slider1 = Slider::make({ 0.0f, 300.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
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
	//---------------------- row Sweep Angle -------------------//
	auto slider2 = Slider::make({ 0.0f, 300.0f });
	slider2->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob2 = TextBlob::make("Sweep Angle");
	textblob2->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val2 = (processing->configuration.sweep_angle - processing->configuration.sweep_angle_limit[0]) / (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
	slider2->set_current_value(current_val2);
	slider2->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.sweep_angle = processing->configuration.sweep_angle_limit[0] + slider->get_current_value() * (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
	});

	auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container2 << std::move(textblob2) << std::move(slider2);
	container2->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Sigma Gradient -------------------//
	auto slider3 = Slider::make({ 0.0f, 300.0f });
	slider3->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob3 = TextBlob::make("Sigma Gradient");
	textblob3->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val3 = (processing->configuration.sigma_gradient - processing->configuration.sigma_gradient_limit[0]) / (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
	slider3->set_current_value(current_val3);
	slider3->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.sigma_gradient = processing->configuration.sigma_gradient_limit[0] + slider->get_current_value() * (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
	});

	auto container3 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container3 << std::move(textblob3) << std::move(slider3);
	container3->set_divisions({ 0.0 , 0.5 , 1.0 });

	//---------------------- row Variance -------------------//
	auto slider4 = Slider::make({ 0.0f, 300.0f });
	slider4->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob4 = TextBlob::make("Variance");
	textblob4->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val4 = (processing->configuration.variance - processing->configuration.variance_limit[0]) / (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
	slider4->set_current_value(current_val4);
	slider4->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.variance = processing->configuration.variance_limit[0] + slider->get_current_value() * (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
	});

	auto container4 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container4 << std::move(textblob4) << std::move(slider4);
	container4->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Disk Ratio -------------------//
	auto slider5 = Slider::make({ 0.0f, 300.0f });
	slider5->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob5 = TextBlob::make("Disk Ratio");
	textblob5->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val5 = (processing->configuration.disk_ratio - processing->configuration.disk_ratio_limit[0]) / (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
	slider5->set_current_value(current_val5);
	slider5->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.disk_ratio = processing->configuration.disk_ratio_limit[0] + slider->get_current_value() * (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
	});

	auto container5 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container5 << std::move(textblob5) << std::move(slider5);
	container5->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row Threshold -------------------//
	auto slider6 = Slider::make({ 0.0f, 300.0f });
	slider6->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob6 = TextBlob::make("Threshold");
	textblob6->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));
	double current_val6 = (processing->configuration.threshold - processing->configuration.threshold_limit[0]) / (double)(processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
	slider6->set_current_value(current_val6);
	slider6->set_callback([&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.threshold = (double)processing->configuration.threshold_limit[0] + slider->get_current_value() * (processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
	});

	auto container6 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container6 << std::move(textblob6) << std::move(slider6);
	container6->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- final -------------------//

	auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*slidercontainer << std::move(container) << std::move(container1) << std::move(container2) << std::move(container3) << std::move(container4) << std::move(container5) << std::move(container6);

	return Overlay::make(std::move(slidercontainer),SK_ColorTRANSPARENT);
}


std::unique_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto button = Button::make("Display Circles",resources);
	button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorCYAN).set_size(SkRect::MakeWH(100, 80));
	button->set_callback([&processing](Button* button, ConfigDraw* config) {
			bool temp = processing->show_circles.load();
			processing->show_circles.store(!temp);
	});

	auto button2 = Button::make("Options",resources);
	button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorCYAN).set_size(SkRect::MakeWH(100, 80));
	button2->set_callback([&processing](Button* button, ConfigDraw* config) {
			config->stack_page->stack(create_filtercontroler_overlay(processing));
	});

	auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*viwers_container << std::move(button) << std::move(button2);
	viwers_container->set_divisions({0.0 , 0.5 , 1.0});
	viwers_container->set_color(SK_ColorTRANSPARENT);

	return Overlay::make(std::move(viwers_container),SK_ColorTRANSPARENT);
}

curan::ui::Page create_main_page(ConfigurationData& data, std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto igtlink_viewer = OpenIGTLinkViewer::make();
	igtlink_viewer->set_size(SkRect::MakeWH(0,0));
	auto igtlink_viewer_pointer = igtlink_viewer.get();

	auto image_display = ImageDisplay::make();
	auto image_display_pointer = image_display.get();

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(igtlink_viewer) << std::move(image_display);
	displaycontainer->set_divisions({ 0.0 , 0.5 , 1.0 });

	auto flag = curan::utilities::Flag::make_shared_flag();

	processing = std::make_shared<ProcessingMessage>(image_display_pointer,igtlink_viewer_pointer, flag, data);
	processing->port = data.port;

	auto lam = [processing](Button* button, ConfigDraw* config) {
		if (!processing->connection_status->value()) {
			curan::utilities::Job val;
			val.description = "connection thread";
			val.function_to_execute = [processing]() {
				processing->communicate();
			};
			curan::utilities::pool->submit(val);
		}
		else {
			processing->attempt_stop();
		}
	};

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	start_connection->set_callback(lam);
	auto start_connection_pointer = start_connection.get();

	auto change_recording_status = [processing](Button* button, ConfigDraw* config) {
		auto val = !processing->should_record.load();
		processing->should_record.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	};

	auto button_start_collection = Button::make("Data Collection",resources);
	button_start_collection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_start_collection->set_callback(change_recording_status);
	auto button_start_collection_pointer = button_start_collection.get();

	auto button_options = Button::make("Options",resources);
	button_options->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_options->set_callback([&processing](Button* button, ConfigDraw* config) {
		config->stack_page->stack(create_options_overlay(processing));
	});

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(button_start_collection) << std::move(button_options);

	processing->button = start_connection_pointer;
	processing->button_start_collection = button_start_collection_pointer;
	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}