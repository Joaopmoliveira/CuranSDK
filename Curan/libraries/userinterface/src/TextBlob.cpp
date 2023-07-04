#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/TextBlob.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {

TextBlob::TextBlob(const std::string& s) : text_to_compile{s} {
	text_color = SK_ColorWHITE;
	background_color = SK_ColorBLACK;
	paint.setStyle(SkPaint::kFill_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(4);
	paint.setColor(background_color);

	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(text_color);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
	text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);
}

std::unique_ptr<TextBlob> TextBlob::make(const std::string& button_text) {
	std::unique_ptr<TextBlob> textblob = std::unique_ptr<TextBlob>(new TextBlob(button_text));
	return textblob;
}

drawablefunction TextBlob::draw() {
	if(!compiled)
		throw std::runtime_error("must compile the button before drawing operations");
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		auto widget_rect = get_position();
		auto size = get_size();

		SkRect drawable = size;
		drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);

		float text_offset_x = drawable.centerX() - widget_rect_text.width() / 2.0f;
		float text_offset_y = drawable.centerY() + widget_rect_text.height() / 2.0f;

		paint.setColor(background_color);
		paint_text.setColor(text_color);
		canvas->drawRect(drawable, paint);
		canvas->drawTextBlob(text, text_offset_x, text_offset_y, paint_text);
	};
	return lamb;
}

callablefunction TextBlob::call() {
	if(!compiled)
		throw std::runtime_error("must compile the button before drawing operations");
	auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utilities::overloaded{
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

void TextBlob::compile(){
	text_font.measureText(text_to_compile.data(), text_to_compile.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(text_to_compile.c_str(), text_font);
	compiled = true;
}

}
}
