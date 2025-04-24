#include "CalibratePages.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Loader.h"


curan::ui::Page create_main_page(ConfigurationData& data, std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto image_display_mean = ImageDisplay::make();
	//image_display_mean->set_color_filter(compliantDicomTransform());
	auto image_display_mean_pointer = image_display_mean.get();

	auto image_display_insertion = ImageDisplay::make();
	//image_display_insertion->set_color_filter(compliantDicomTransform());
	auto image_display_insertion_pointer = image_display_insertion.get();

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(image_display_mean) << std::move(image_display_insertion);

	processing = std::make_shared<ProcessingMessage>(image_display_mean_pointer,image_display_insertion_pointer, data,nullptr);
	processing->port = data.port;

	auto start_connection_callback = [&data,processing,&resources](Button* button, Press press ,ConfigDraw* config) {
		if (!processing->connection_status.value()) {
			curan::utilities::Job val{"connection thread",[processing]() { processing->communicate();}};
			data.shared_pool->submit(val);
			processing->config_draw->stack_page->stack(Loader::make("human_robotics_logo.jpeg",resources));
		}
		else {
			processing->attempt_stop();
		}
	};

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
	start_connection->add_press_call(start_connection_callback);
	auto start_connection_pointer = start_connection.get();

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection);
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}