#include "CalibratePages.h"

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(ProcessingMessage* processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	Container::InfoLinearContainer infocontainer;
	infocontainer.paint_layout = paint_square2;
	infocontainer.arrangement = curan::ui::Arrangement::HORIZONTAL;

	Slider::Info infor{};
	infor.click_color = SK_ColorLTGRAY;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorDKGRAY;
	infor.sliderColor = SK_ColorGRAY;
	infor.paintButton = paint_square;
	infor.size = SkRect::MakeWH(200, 40);
	infor.limits = { 0.0f, 300.0f };
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.minimum_radius = processing->configuration.minimum_radius_limit[0] + slider->get_current_value() * (processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
	};
	std::shared_ptr<Slider> button = Slider::make(infor);
	double current_val = (processing->configuration.minimum_radius - processing->configuration.minimum_radius_limit[0]) / (processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
	button->set_current_value(current_val);

	TextBlob::Info infotext;
	infotext.button_text = "Minimum Radius";
	infotext.paint = paint_square;
	infotext.paintText = paint_text;
	infotext.size = SkRect::MakeWH(200, 40);
	infotext.textFont = text_font;
	std::shared_ptr<TextBlob> text = TextBlob::make(infotext);

	infocontainer.layouts = { text,button };
	std::shared_ptr<Container> container = Container::make(infocontainer);

	infotext.button_text = "Maximum Radius";
	std::shared_ptr<TextBlob> text1 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.maximum_radius = processing->configuration.maximum_radius_limit[0] + slider->get_current_value() * (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
	};
	std::shared_ptr<Slider> button1 = Slider::make(infor);
	double current_val1 = (processing->configuration.maximum_radius - processing->configuration.maximum_radius_limit[0]) / (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
	button1->set_current_value(current_val1);
	infocontainer.layouts = { text1,button1 };
	std::shared_ptr<Container> container1 = Container::make(infocontainer);

	infotext.button_text = "Sweep Angle";
	std::shared_ptr<TextBlob> text2 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.sweep_angle = processing->configuration.sweep_angle_limit[0] + slider->get_current_value() * (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
	};
	std::shared_ptr<Slider> button2 = Slider::make(infor);
	double current_val2 = (processing->configuration.sweep_angle - processing->configuration.sweep_angle_limit[0]) / (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
	button2->set_current_value(current_val2);
	infocontainer.layouts = { text2,button2 };
	std::shared_ptr<Container> container2 = Container::make(infocontainer);

	infotext.button_text = "Sigma Gradient";
	std::shared_ptr<TextBlob> text3 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.sigma_gradient = processing->configuration.sigma_gradient_limit[0] + slider->get_current_value() * (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
	};
	std::shared_ptr<Slider> button3 = Slider::make(infor);
	double current_val3 = (processing->configuration.sigma_gradient - processing->configuration.sigma_gradient_limit[0]) / (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
	button3->set_current_value(current_val3);
	infocontainer.layouts = { text3,button3 };
	std::shared_ptr<Container> container3 = Container::make(infocontainer);

	infotext.button_text = "Variance";
	std::shared_ptr<TextBlob> text4 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.variance = processing->configuration.variance_limit[0] + slider->get_current_value() * (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
	};
	std::shared_ptr<Slider> button4 = Slider::make(infor);
	double current_val4 = (processing->configuration.variance - processing->configuration.variance_limit[0]) / (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
	button4->set_current_value(current_val4);
	infocontainer.layouts = { text4,button4 };
	std::shared_ptr<Container> container4 = Container::make(infocontainer);

	infotext.button_text = "Disk Ratio";
	std::shared_ptr<TextBlob> text5 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.disk_ratio = processing->configuration.disk_ratio_limit[0] + slider->get_current_value() * (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
	};
	std::shared_ptr<Slider> button5 = Slider::make(infor);
	double current_val5 = (processing->configuration.disk_ratio - processing->configuration.disk_ratio_limit[0]) / (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
	button5->set_current_value(current_val5);
	infocontainer.layouts = { text5,button5 };
	std::shared_ptr<Container> container5 = Container::make(infocontainer);

	infotext.button_text = "Threshold";
	std::shared_ptr<TextBlob> text6 = TextBlob::make(infotext);
	infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
		processing->configuration.threshold = (double)processing->configuration.threshold_limit[0] + slider->get_current_value() * (processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
	};
	std::shared_ptr<Slider> button6 = Slider::make(infor);
	double current_val6 = (processing->configuration.threshold - processing->configuration.threshold_limit[0]) / (double)(processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
	button6->set_current_value(current_val6);
	infocontainer.layouts = { text6,button6 };
	std::shared_ptr<Container> container6 = Container::make(infocontainer);

	infocontainer.arrangement = curan::ui::Arrangement::VERTICAL;
	infocontainer.layouts = { container,container1,container2,container3,container4,container5,container6 };
	std::shared_ptr<Container> containerotions = Container::make(infocontainer);

	Overlay::Info information;
	information.backgroundcolor = SK_ColorTRANSPARENT;
	information.contained = containerotions;
	return Overlay::make(information);
}


std::unique_ptr<curan::ui::Overlay> create_options_overlay(ProcessingMessage* processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	std::shared_ptr<Button> display_type;
	{
		Button::Info infor{ resources };
		infor.button_text = "Display Circles";
		infor.click_color = SK_ColorLTGRAY;
		infor.hover_color = SK_ColorCYAN;
		infor.waiting_color = SK_ColorDKGRAY;
		infor.icon_identifier = "";
		infor.paintButton = paint_square;
		infor.paintText = paint_text;
		infor.size = SkRect::MakeWH(200, 80);
		infor.textFont = text_font;
		infor.callback = [&processing](Button* button, ConfigDraw* config) {
			bool temp = processing->show_circles.load();
			processing->show_circles.store(!temp);
		};
		display_type = Button::make(infor);
	}

	std::shared_ptr<Button> options;

	{
		Button::Info infor{ resources };
		infor.button_text = "Options";
		infor.click_color = SK_ColorLTGRAY;
		infor.hover_color = SK_ColorCYAN;
		infor.waiting_color = SK_ColorDKGRAY;
		infor.icon_identifier = "";
		infor.paintButton = paint_square;
		infor.paintText = paint_text;
		infor.size = SkRect::MakeWH(200, 80);
		infor.textFont = text_font;
		infor.callback = [&processing](Button* button, ConfigDraw* config) {
			auto overlay = create_filtercontroler_overlay(processing);
			config->stack_page->stack(overlay);
		};
		options = Button::make(infor);
	}


	Container::InfoLinearContainer info;
	info.paint_layout = paint_square2;
	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.5 , 1.0 };
	info.layouts = { display_type,options };
	std::shared_ptr<Container> viwers_container = Container::make(info);

	Overlay::Info information;
	information.backgroundcolor = SK_ColorTRANSPARENT;
	information.contained = viwers_container;
	return Overlay::make(information);
}


curan::ui::Page create_main_page(ConfigurationData& data, ProcessingMessage* processing,curan::ui::IconResources& resources) {
	using namespace curan::ui;

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
	std::shared_ptr<Container> viwers_container = Container::make(info);

	auto flag = curan::utilities::Flag::make_shared_flag();

	processing = std::make_shared<ProcessingMessage>(processed_viwer, open_viwer, flag, data);
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

	Button::Info infor{ resources };
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
	std::shared_ptr<Button> start_connection = Button::make(infor);

	auto change_recording_status = [processing](Button* button, ConfigDraw* config) {
		auto val = !processing->should_record.load();
		processing->should_record.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	};

	infor.button_text = "Data Collection";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorBLACK;
	infor.callback = change_recording_status;
	infor.size = SkRect::MakeWH(200, 80);
	std::shared_ptr<Button> button_start_collection = Button::make(infor);

	infor.button_text = "Options";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorBLACK;
	infor.size = SkRect::MakeWH(200, 80);

	infor.callback = [&processing](Button* button, ConfigDraw* config) {
		auto overlay = create_options_overlay(processing);
		config->stack_page->stack(overlay);
	};
	std::shared_ptr<Button> button_options = Button::make(infor);

	info.layouts = { start_connection,button_start_collection,button_options };
	std::shared_ptr<Container> button_container = Container::make(info);

	processing->button = start_connection;
	processing->button_start_collection = button_start_collection;
	start_connection->set_waiting_color(SK_ColorRED);

	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.1 , 1.0 };
	info.layouts = { button_container,viwers_container };
	std::shared_ptr<Container> container = Container::make(info);

	Page::Info information;
	information.backgroundcolor = SK_ColorBLACK;
	information.contained = container;
	std::shared_ptr<Page> page = Page::make(information);
	return page;
}