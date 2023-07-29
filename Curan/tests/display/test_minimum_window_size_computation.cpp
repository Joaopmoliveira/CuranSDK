#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include <iostream>

void create_horizontal_layout(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0f , 0.33333f , 0.66666f , 1.0f });
	container->compile();

    auto minimum_size = container->minimum_size();
    std::printf("expected\n\twidth: (%f) height: (%f)\n",300.0,200.0);
    std::printf("real\n\twidth: (%f) height: (%f)\n",minimum_size.width(),minimum_size.height());
}

void create_vertical_layout(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0f , 0.33333f , 0.66666f , 1.0f });
	container->compile();

    auto minimum_size = container->minimum_size();
    std::printf("expected\n\twidth: (%f) height: (%f)\n",100.0,600.0);
    std::printf("real\n\twidth: (%f) height: (%f)\n",minimum_size.width(),minimum_size.height());
}

void create_variable_layout(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::VARIABLE_CONTAINER,curan::ui::Container::Arrangement::UNDEFINED);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_variable_layout({ SkRect::MakeLTRB(0.0f,0.0f,0.3333f,1.0f),SkRect::MakeLTRB(0.3333f,0.0f,0.6666f,1.0f),SkRect::MakeLTRB(0.6666f,0.0f,1.0f,1.0f) });

	container->compile();

    auto minimum_size = container->minimum_size();
    std::printf("expected\n\twidth: (%f) height: (%f)\n",300.0,200.0);
    std::printf("real\n\twidth: (%f) height: (%f)\n",minimum_size.width(),minimum_size.height());
}

void create_nested_layout_propagate(curan::ui::IconResources& resources){
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button1 = button.get();

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button2 = button2.get();

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button3 = button3.get();


	std::unique_ptr<curan::ui::Button> button4 = curan::ui::Button::make("Touch4!",resources);
	button4->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button4 = button4.get();

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0f , 0.33333f , 0.66666f , 1.0f });

	std::unique_ptr<curan::ui::Container> container2 = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
	*container2 << std::move(container) << std::move(button4);
	container2->set_divisions({  0.0f , 0.5f , 1.0f });

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container2->set_position(my_small_window);
	container2->compile();
	container2->framebuffer_resize();

    auto minimum_size = container->minimum_size();
    std::printf("expected\n\twidth: (%f) height: (%f)\n",200.0f,600.0f);
    std::printf("real\n\twidth: (%f) height: (%f)\n",minimum_size.width(),minimum_size.height());
};

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::cout << "\n\n============ create_horizontal_layout ============\n\n";
		create_horizontal_layout(resources);
		std::cout << "\n\n============ create_vertical_layout ============\n\n";
		create_vertical_layout(resources);
		std::cout << "\n\n============ create_variable_layout ============\n\n";
		create_variable_layout(resources);
	}
	catch (std::exception& e) {
		std::cout << "Failed : " << e.what() << std::endl;
		return 1;
	}
}