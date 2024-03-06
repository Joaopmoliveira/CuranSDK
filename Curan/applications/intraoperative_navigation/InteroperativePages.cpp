#include "InteroperativePages.h"

ProcessingMessage create_main_page(curan::ui::IconResources& resources,std::unique_ptr<curan::ui::Container>& container, InputImageType::Pointer in_volume) {
	using namespace curan::ui;

	auto igtlink_viewer = OpenIGTLinkViewer::make();
	igtlink_viewer->set_size(SkRect::MakeWH(0,0));
	auto igtlink_viewer_pointer = igtlink_viewer.get();

	auto image_display = ImageDisplay::make();
	auto image_display_pointer = image_display.get();

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(igtlink_viewer) << std::move(image_display);
	displaycontainer->set_divisions({ 0.0 , 0.5 , 1.0 });

	ProcessingMessage processing{image_display_pointer,in_volume};

	auto start_connection_callback = [&](Button* button, Press press ,ConfigDraw* config) {
		if (!processing->connection_status.value()) {
			curan::utilities::Job val{"connection thread",[processing]() { processing->communicate();}};
			data.shared_pool->submit(val);
		}
		else {
			processing->attempt_stop();
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
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

    container = widgetcontainer;

    return std::move(processing);
}