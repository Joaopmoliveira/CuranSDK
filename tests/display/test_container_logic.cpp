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
	
	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Rect1 left: 0       top: 0  right: 0.33333 bottom: 1 \n";
	std::cout << "Rect2 left: 0.33333 top: 0  right: 0.66666 bottom: 1 \n";
	std::cout << "Rect3 left: 0.66666 top: 0  right: 1.00000 bottom: 1 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
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

	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\nRect1 left: 0.0 top: 0.0  right: 1.0 bottom: 1.0 \n";
			   std::cout << "Rect2 left: 0.0 top: 0.3333  right: 1.0 bottom: 0.6666 \n";
			   std::cout << "Rect3 left: 0.0 top: 0.6666  right: 1.0 bottom: 1.0 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
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

	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\nRect1 left: 0 top: 0  right: 0.33333 bottom: 1 \n";
	std::cout << "Rect2 left: 0.33333 top: 0  right: 0.66666 bottom: 1 \n";
	std::cout << "Rect3 left: 0.66666 top: 0  right: 1.00000 bottom: 1 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
}


void create_horizontal_layout_propagate(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button1 = button.get();

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button2 = button2.get();

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button3 = button3.get();

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0 , 0.33333 , 0.66666 , 1.0 });

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container->set_position(my_small_window);

	container->compile();

	container->framebuffer_resize(my_small_window);

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left:  50 top:  50  right: 350 bottom: 950 \n";
	std::cout << "Button2 left: 350 top:  50  right: 650 bottom: 950 \n";
	std::cout << "Button3 left: 650 top:  50  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = temporary_storage_button1->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = temporary_storage_button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = temporary_storage_button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
}


void create_vertical_layout_propagate(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button1 = button.get();

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button2 = button2.get();

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));
	auto temporary_storage_button3 = button3.get();

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0f , 0.33333f , 0.66666f , 1.0f });

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container->set_position(my_small_window);

	container->compile();

	container->framebuffer_resize(my_small_window);

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left: 50 top:  50  right: 950 bottom: 350 \n";
	std::cout << "Button2 left: 50 top: 350  right: 950 bottom: 650 \n";
	std::cout << "Button3 left: 50 top: 650  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = temporary_storage_button1->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = temporary_storage_button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = temporary_storage_button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
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
	container2->framebuffer_resize(my_small_window);

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left:  50 top:  50  right: 500 bottom: 350 \n";
	std::cout << "Button2 left:  50 top: 350  right: 500 bottom: 650 \n";
	std::cout << "Button3 left:  50 top: 650  right: 500 bottom: 950 \n";
	std::cout << "Button4 left: 500 top:  50  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = temporary_storage_button1->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = temporary_storage_button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = temporary_storage_button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
	auto pos4 = temporary_storage_button4->get_position();
	std::cout << "Button4 left: " << pos4.fLeft << " top: " << pos4.fTop << " right: " << pos4.fRight << " bottom: " << pos4.fBottom << "\n";
};

void test_linearization(curan::ui::IconResources& resources) {
	std::unique_ptr<curan::ui::Button> button = curan::ui::Button::make("Touch!",resources);
	button->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button2 = curan::ui::Button::make("Touch2!",resources);
	button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button3 = curan::ui::Button::make("Touch3!",resources);
	button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Button> button4 = curan::ui::Button::make("Touch4!",resources);
	button4->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(100,200));

	std::unique_ptr<curan::ui::Container> container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
	*container << std::move(button) << std::move(button2) << std::move(button3);
	container->set_divisions({ 0.0 , 0.33333 , 0.66666 , 1.0 });

	std::unique_ptr<curan::ui::Container> container2 = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
	*container2 << std::move(container) << std::move(button4);
	container2->set_divisions({  0.0 , 0.5 , 1.0 });

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);

	container2->set_position(my_small_window);
	container2->compile();
	container2->framebuffer_resize(my_small_window);

	std::vector<curan::ui::drawablefunction> temp_draw;
	std::vector<curan::ui::callablefunction> temp_call;
	container2->linearize_container(temp_draw,temp_call);

	std::cout << "expected size drawable: (6) real size: (" << temp_draw.size() << ")\n";
	std::cout << "expected size callable: (6) real size: (" << temp_call.size() << ")\n";
}

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
		std::cout << "\n\n============ create_horizontal_layout_propagate ============\n\n";
		create_horizontal_layout_propagate(resources);
		std::cout << "\n\n============ create_vertical_layout_propagate ============\n\n";
		create_vertical_layout_propagate(resources);
		std::cout << "\n\n============ create_nested_layout_propagate ============\n\n";
		create_nested_layout_propagate(resources);
		std::cout << "\n\n============ test_linearization ============\n\n";
		test_linearization(resources);
	}
	catch (std::exception& e) {
		std::cout << "Failed : " << e.what() << std::endl;
		return 1;
	}
}