#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/TextBlob.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {

TextBlob::TextBlob(Info& info) : Drawable{ info.size } {
	paint = info.paint;
	paint_text = info.paintText;
	text_font = info.textFont;
	text_font.measureText(info.button_text.data(), info.button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(info.button_text.c_str(), text_font);
}

std::shared_ptr<TextBlob> TextBlob::make(Info& info) {
	return std::make_shared<TextBlob>(info);
}

drawablefunction TextBlob::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		auto widget_rect = get_position();
		auto size = get_size();

		SkRect drawable = size;
		drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);

		float text_offset_x = drawable.centerX() - widget_rect_text.width() / 2.0f;
		float text_offset_y = drawable.centerY() + widget_rect_text.height() / 2.0f;

		canvas->drawRect(drawable, paint);
		canvas->drawTextBlob(text, text_offset_x, text_offset_y, paint_text);
	};
	return lamb;
}

callablefunction TextBlob::call() {
	auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utils::overloaded{
			[this](Empty arg) {

			},
			[this,&interacted](Move arg) {

			},
			[this,&interacted](Press arg) {

			},
			[this](Scroll arg) {;

			},
			[this,&interacted](Unpress arg) {

			},
			[this](Key arg) {

			},
			[this](ItemDropped arg) {;

			} },
			sig);
		return interacted;
	};
	return lamb;
}

void TextBlob::framebuffer_resize() {

}

}
}
