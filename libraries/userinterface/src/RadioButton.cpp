#include "userinterface/widgets/RadioButton.h"

namespace curan {
namespace ui {

RadioButton::Info::Info() {
	
}

bool RadioButton::Info::validate() {

}

RadioButton::RadioButton(Info& info) {
	color = info.color;
	text_font = info.text_font;
	size = info.size;
	background_color = info.background_color;
	is_exclusive = info.is_exclusive;

	paint.setStyle(SkPaint::kStroke_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(1);
	paint.setColor(color);

	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(0.5);
	paint_text.setColor(color);

	switch (info.layout) {
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
