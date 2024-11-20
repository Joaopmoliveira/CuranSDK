#include "CalibratePages.h"

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;
	//---------------------- row Minimum Radius -------------------//
	auto slider = Slider::make({ 0.0f, 300.0f });
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	//---------------------- row Sweep Angle -------------------//
	auto slider2 = Slider::make({ 0.0f, 300.0f });
	slider2->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	slider3->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	slider4->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	slider5->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	slider6->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 40));
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
	slidercontainer->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(246, 246, 246)});
	return Overlay::make(std::move(slidercontainer),SK_ColorTRANSPARENT,true);
}


std::unique_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto button = Button::make("Display Circles",resources);
	button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	button->add_press_call([&processing](Button* button, Press press ,ConfigDraw* config) {
			bool temp = processing->show_circles.load();
			processing->show_circles.store(!temp);
	});

	auto button2 = Button::make("Options",resources);
	button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	button2->add_press_call([&processing,&resources](Button* button, Press press ,ConfigDraw* config) {
			config->stack_page->stack(create_filtercontroler_overlay(processing,resources));
	});

	auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*viwers_container << std::move(button) << std::move(button2);
	viwers_container->set_divisions({0.0 , 0.5 , 1.0});
	viwers_container->set_color(SK_ColorTRANSPARENT);
	
	return Overlay::make(std::move(viwers_container),SkColorSetARGB(10,125,125,125),true);
}

curan::ui::Page create_main_page(ConfigurationData& data, std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto igtlink_viewer = OpenIGTLinkViewer::make();
	igtlink_viewer->set_size(SkRect::MakeWH(0,0));
	auto igtlink_viewer_pointer = igtlink_viewer.get();
	uint_least8_t dicom_compliant_conversion[256] = {0 ,30 ,33 ,35 ,37 ,39 ,40 ,41 ,43 ,44 ,45 ,46 ,47 ,48 ,48 ,49 ,50 ,51 ,51 ,52 ,53 ,54 ,55 ,55 ,56 ,57 ,57 ,58 ,59 ,60 ,60 ,61 ,62 ,62 ,63 ,64 ,64 ,65 ,65 ,66 ,67 ,67 ,68 ,69 ,69 ,70 ,71 ,71 ,72 ,73 ,73 ,74 ,75 ,76 ,76 ,77 ,77 ,78 ,79 ,80 ,80 ,80 ,81 ,82 ,83 ,83 ,84 ,84 ,85 ,86 ,86 ,87 ,88 ,88 ,89 ,90 ,90 ,91 ,92 ,93 ,93 ,94 ,95 ,95 ,96 ,96 ,97 ,98 ,98 ,99 ,100 ,101 ,101 ,102 ,103 ,103 ,104 ,105 ,106 ,106 ,107 ,108 ,109 ,109 ,110 ,111 ,111 ,112 ,113 ,113 ,114 ,115 ,116 ,116 ,117 ,118 ,119 ,119 ,120 ,121 ,122 ,123 ,123 ,124 ,125 ,126 ,126 ,127 ,128 ,128 ,129 ,130 ,131 ,132 ,132 ,133 ,134 ,135 ,136 ,136 ,137 ,138 ,139 ,140 ,141 ,141 ,142 ,143 ,144 ,145 ,145 ,146 ,147 ,148 ,149 ,150 ,151 ,151 ,152 ,153 ,154 ,155 ,156 ,157 ,158 ,158 ,159 ,160 ,161 ,162 ,163 ,164 ,164 ,166 ,167 ,167 ,169 ,170 ,170 ,171 ,172 ,173 ,174 ,175 ,176 ,176 ,178 ,178 ,180 ,181 ,182 ,183 ,184 ,184 ,186 ,186 ,188 ,188 ,189 ,191 ,191 ,192 ,194 ,194 ,196 ,197 ,197 ,199 ,200 ,201 ,202 ,203 ,204 ,205 ,207 ,207 ,208 ,209 ,210 ,212 ,213 ,214 ,215 ,216 ,217 ,218 ,220 ,221 ,222 ,223 ,224 ,225 ,226 ,228 ,229 ,230 ,231 ,233 ,234 ,235 ,236 ,238 ,239 ,240 ,241 ,242 ,244 ,245 ,246 ,248 ,249 ,250 ,251 ,253 ,254 ,255};

	auto image_display = ImageDisplay::make();
	auto image_display_pointer = image_display.get();
	image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
	igtlink_viewer->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(igtlink_viewer) << std::move(image_display);
	displaycontainer->set_divisions({ 0.0 , 0.5 , 1.0 });

	processing = std::make_shared<ProcessingMessage>(image_display_pointer,igtlink_viewer_pointer, data);
	processing->port = data.port;

	auto start_connection_callback = [&data,processing](Button* button, Press press ,ConfigDraw* config) {
		if (!processing->connection_status.value()) {
			curan::utilities::Job val{"connection thread",[processing]() { processing->communicate();}};
			data.shared_pool->submit(val);
		}
		else {
			processing->attempt_stop();
			processing->connection_status.set(false);
		}
	};

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	start_connection->add_press_call(start_connection_callback);
	auto start_connection_pointer = start_connection.get();


	auto button_start_collection = Button::make("Data Collection",resources);
	button_start_collection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_start_collection->add_press_call([processing](Button* button,Press press , ConfigDraw* config) {
		auto val = !processing->should_record.load();
		processing->should_record.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	});
	processing->button_start_collection = button_start_collection.get();

	auto button_options = Button::make("Options",resources);
	button_options->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	button_options->add_press_call([&processing,&resources](Button* button,Press press , ConfigDraw* config) {
		config->stack_page->stack(create_options_overlay(processing,resources));
	});

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(button_start_collection) << std::move(button_options);
	buttoncontainer->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(246, 246, 246)});
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}