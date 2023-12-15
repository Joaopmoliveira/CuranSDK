#include "userinterface/widgets/Button.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

Button::Button(const std::string& in_button_text,IconResources& system_icons) : button_text{in_button_text} , system_icons{system_icons}{
	hover_color = SK_ColorDKGRAY;
	waiting_color = SK_ColorGRAY;
	click_color = SK_ColorCYAN;
	text_color = SK_ColorWHITE;

	paint.setStyle(SkPaint::kFill_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(4);
	paint.setColor(hover_color);

	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(text_color);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
}

Button::Button(const std::string& in_button_text,const std::string& in_icon_identifier,IconResources& system_icons) : icon_identifier{in_icon_identifier}, button_text{in_button_text} , system_icons{system_icons}{
	hover_color = SK_ColorDKGRAY;
	waiting_color = SK_ColorGRAY;
	click_color = SK_ColorCYAN;
	text_color = SK_ColorWHITE;

	paint.setStyle(SkPaint::kFill_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(4);
	paint.setColor(hover_color);

	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(text_color);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
}


Button::~Button(){

}

std::unique_ptr<Button> Button::make(const std::string& button_text,IconResources& system_icons){
	std::unique_ptr<Button> button = std::unique_ptr<Button>(new Button{button_text,system_icons});
	return button;
}	

std::unique_ptr<Button> Button::make(const std::string& button_text,const std::string& icon_identifier,IconResources& system_icons){
	std::unique_ptr<Button> button = std::unique_ptr<Button>(new Button{button_text,icon_identifier,system_icons});
	return button;
}	

drawablefunction Button::draw(){
if(!compiled)
	throw std::runtime_error("must compile the button before drawing operations");
	auto lamb = [this](SkCanvas* canvas) {
		switch (current_state) {
		case ButtonStates::WAITING:
			paint.setColor(get_waiting_color());
			break;
		case ButtonStates::HOVER:
			paint.setColor(get_hover_color());
			break;
		case ButtonStates::PRESSED:
			paint.setColor(get_click_color());
			break;
		}
		auto widget_rect = get_position();
		auto size = get_size();

		SkRect drawable = size;
		drawable.offsetTo(widget_rect.centerX()- drawable.width()/2.0f, widget_rect.centerY()- drawable.height() / 2.0f);

		float text_offset_x = drawable.centerX() - widget_rect_text.width() / 2.0f;
		float text_offset_y = drawable.centerY() + widget_rect_text.height() / 2.0f;
				
		canvas->drawRect(drawable, paint);
		canvas->drawTextBlob(text, text_offset_x, text_offset_y, paint_text);

		if (icon_data.get() != nullptr) {
			float image_width = static_cast<float>(icon_data->width());
			float image_height = static_cast<float>(icon_data->height());
			float current_selected_width = drawable.width();
			float current_selected_height = drawable.height() - widget_rect_text.height();

			float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);

			float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + drawable.x();
			float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + drawable.y();

			SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
			canvas->drawImageRect(icon_data, current_selected_image_rectangle, opt);
		}
	};
	return lamb;
}

callablefunction Button::call(){
if(!compiled)
	throw std::runtime_error("must compile the button before drawing operations");
auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utilities::overloaded{
			[this,config](Empty arg) {

			},
			[this,&interacted,config](Move arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos)) {
					if(previous_state != ButtonStates::PRESSED)
						current_state_local = ButtonStates::HOVER;
				}
				else {
					current_state_local = ButtonStates::WAITING;
				}
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this,&interacted,config](Press arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos,arg.ypos)) {
					current_state_local = ButtonStates::PRESSED;	
					for(const auto& localcall : callbacks_press)
                		localcall(this,arg,config);				
				}
				else
					current_state_local = ButtonStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this,config](Scroll arg) {;

			},
			[this,&interacted,config](Unpress arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos))
					current_state_local = ButtonStates::HOVER;
				else
					current_state_local = ButtonStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this](Key arg) {

			},
			[this](ItemDropped arg) {;

			}},
			sig);
			return interacted;
		};
	return lamb;
}

void Button::compile(){
	std::lock_guard<std::mutex> g{ get_mutex() };

	auto text_font = SkFont(typeface, font_size, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	text_font.measureText(button_text.data(), button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(button_text.c_str(), text_font);

	if (system_icons.is_loaded() && icon_identifier.size()>0) {
		sk_sp<SkImage> image;
		system_icons.get_icon(image,icon_identifier);
		icon_data = image;
	}
	compiled = true;
}

}
}