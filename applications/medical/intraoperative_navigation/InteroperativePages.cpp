#include "InteroperativePages.h"
#include "utils/Job.h"

std::unique_ptr<curan::ui::Container> create_main_page(curan::ui::IconResources& resources, ProcessingMessage& proc) {
	using namespace curan::ui;

	auto image_display = ImageDisplay::make();
	proc.processed_viwer = image_display.get();

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(image_display);

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorDKGRAY)
                    .set_waiting_color(SK_ColorBLACK)
                    .set_size(SkRect::MakeWH(100, 80));
	start_connection->add_press_call([&](Button* button, Press press ,ConfigDraw* config) {
		std::cout << "button clicked" << std::endl;
		std::cout << "proc" << std::endl;
		if (!proc.connection_status) {
			std::cout << "proc" << std::endl;
			proc.shared_pool->submit("connection thread",[&]() { try{proc.communicate();} catch(...){std::cout << "Communication exception throw" << std::endl;}});
		}
		else {
			proc.attempt_stop();
		}
	});
	auto start_connection_pointer = start_connection.get();

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection);
	proc.button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

    return std::move(widgetcontainer);
}