#include "userinterface/widgets/RadioButton.h"

namespace curan {
namespace ui {

RadioButton::Info::Info() {
	
}

bool RadioButton::Info::validate() {

}

RadioButton::RadioButton(Info& info) {
	hover_color = info.hover_color;
	click_color = info.click_color;
	waiting_color = info.waiting_color;
	text_font = info.text_font;
	is_exclusive = info.is_exclusive;

	paint.setStyle(SkPaint::kStroke_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(1);
	paint.setColor(waiting_color);

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
			text_font.measureText(info.options[index].data(), info.options[index].size(), SkTextEncoding::kUTF8, &item.text_size);;
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
			item.text = SkTextBlob::MakeFromString(info.options[index].c_str(), text_font);;
			text_font.measureText(info.options[index].data(), info.options[index].size(), SkTextEncoding::kUTF8, &item.text_size);;
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

}

drawablefunction RadioButton::draw() {

}

callablefunction RadioButton::call() {

}

void RadioButton::framebuffer_resize() {

}

}
}
