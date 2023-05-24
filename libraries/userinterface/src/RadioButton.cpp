#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/RadioButton.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {

RadioButton::Info::Info() {
	
}

bool RadioButton::Info::validate() {
	return true;
}

RadioButton::RadioButton(Info& info) : Drawable{ info.size }{
	hover_color = info.hover_color;
	click_color = info.click_color;
	waiting_color = info.waiting_color;
	text_font = info.text_font;
	is_exclusive = info.is_exclusive;

	paint_button = info.paintButton;
	paint_text = info.paintText;

	switch (info.arrangement) {
	case Arrangement::HORIZONTAL:
	{
		double normalized_dimensions = 1.0 / info.options.size();
		double left_coordinate = 0.0;
		for (int index = 0; index < info.options.size(); ++index) {
			RadioItem item;
			item.normalized_position = SkRect::MakeXYWH(left_coordinate, 0, normalized_dimensions, 1);
			item.text = SkTextBlob::MakeFromString(info.options[index].c_str(), text_font);;
			radio_items.push_back(item);
			left_coordinate += normalized_dimensions;
		}
		break;
	}
	case Arrangement::VERTICAL:
	{
		double normalized_dimensions = 1.0 / info.options.size();
		double left_coordinate = 0.0;
		for (int index = 0; index < info.options.size(); ++index) {
			RadioItem item;
			item.normalized_position = SkRect::MakeXYWH(0, left_coordinate, 1, normalized_dimensions);
			item.text = SkTextBlob::MakeFromString(info.options[index].c_str(), text_font);
			radio_items.push_back(item);
			left_coordinate += normalized_dimensions;
		}
		break;
	}
	default:

		break;
	}
}

std::shared_ptr<RadioButton> RadioButton::make(Info& info) {
	return std::make_shared<RadioButton>(info);
}

drawablefunction RadioButton::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		auto widget_rect = get_position();
		auto size = get_size();
		SkRect sized_rectangle = SkRect::MakeXYWH(widget_rect.centerX() - size.width() / 2, widget_rect.centerY() - size.height() / 2, size.width(), size.height());
		paint_button.setColor(waiting_color);
		canvas->drawRect(sized_rectangle, paint_button);

		for (auto& component : radio_items){
			canvas->drawRect(component.item_position, paint_button);

			if (component.is_selected) {
				canvas->drawLine({ component.item_position.fLeft,component.item_position.fTop }, { component.item_position.fRight,component.item_position.fBottom }, paint_button);
				canvas->drawLine({ component.item_position.fRight,component.item_position.fTop }, { component.item_position.fLeft,component.item_position.fBottom }, paint_button);
			}
			auto textbounds = component.text->bounds();
			canvas->drawTextBlob(component.text, component.item_position.fRight + RADIO_BUTTON_DIMENSION, component.item_position.centerY() + textbounds.height() / 2, paint_text);
		}
	};
	return lamb;
}

callablefunction RadioButton::call() {
	auto lamb = [this](Signal sig, ConfigDraw* config) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		bool interacted = false;
		std::visit(utilities::overloaded{
			[this](Empty arg) {

			},
			[this](Move arg) {

			},
			[this,&interacted](Press arg) {
				if (interacts(arg.xpos, arg.ypos)) {
					bool was_item_clicked = false;
					for (auto& component : radio_items) {
						if (((arg.xpos > component.item_position.fLeft) && (arg.xpos < component.item_position.fRight))
							&& ((arg.ypos > component.item_position.fTop) && (arg.ypos < component.item_position.fBottom)))
						{
							was_item_clicked = true;
							component.is_selected = !component.is_selected;
						}
						if (is_exclusive && !was_item_clicked)
							component.is_selected = false;
						was_item_clicked = false;
				}
			};
			},
			[this](Scroll arg) {;

			},
			[this](Key arg) {

			},
			[this](Unpress arg) {;

			},
			[this](ItemDropped arg) {;

			} }, sig);
		return interacted;
	};
	return lamb;
}

void RadioButton::framebuffer_resize() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	auto widget_rect = get_position();
	auto size = get_size();
	SkRect sized_rectangle = SkRect::MakeXYWH(widget_rect.centerX() - size.width() / 2, widget_rect.centerY() - size.height() / 2, size.width(), size.height());
	for (auto& component : radio_items) {
		SkRect temp = SkRect::MakeXYWH(sized_rectangle.width() * component.normalized_position.x() + sized_rectangle.x(),
			sized_rectangle.height() * component.normalized_position.y() + sized_rectangle.y(),
			sized_rectangle.width() * component.normalized_position.width(),
			sized_rectangle.height() * component.normalized_position.height());

		component.item_position = SkRect::MakeXYWH(temp.fLeft + RADIO_BUTTON_DIMENSION,
			temp.centerY() - (RADIO_BUTTON_DIMENSION) / 2.0,
			RADIO_BUTTON_DIMENSION,
			RADIO_BUTTON_DIMENSION);
}
}

}
}
