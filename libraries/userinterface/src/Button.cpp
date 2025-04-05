#include "userinterface/widgets/Button.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/definitions/Interactive.h"

#include <iostream>
#include "utils/Logger.h"

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

	typeface = defaultTypeface();
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

	typeface = defaultTypeface();
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
if(!compiled) {
	utilities::print<utilities::Severity::major_failure>("button{:d} : was not compiled\n",(uintptr_t)this);
	throw std::runtime_error("must compile the button before drawing operations"); 
}
	
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
			canvas->drawTextBlob(text, text_offset_x, current_selected_image_rectangle.bottom(), paint_text);
		} else {
			canvas->drawTextBlob(text, text_offset_x, text_offset_y, paint_text);
		}
	};
	return lamb;
}

callablefunction Button::call(){
if(!compiled){
	utilities::print<utilities::Severity::major_failure>("button{:d} : was not compiled\n",(uintptr_t)this);
	throw std::runtime_error("must compile the button before drawing operations");
}
auto lamb = [this](Signal sig, ConfigDraw* config) {
		auto check_inside_fixed_area = [this](double x,double y){ 
			auto widget_rect = get_position();
			auto size = get_size();
			SkRect drawable = size;
			drawable.offsetTo(widget_rect.centerX()- drawable.width()/2.0f, widget_rect.centerY()- drawable.height() / 2.0f);
			return drawable.contains(x,y); 
		};
		interpreter.process(check_inside_fixed_area,check_inside_fixed_area,sig);
		
		if(interpreter.check(INSIDE_FIXED_AREA | MOUSE_CLICKED_LEFT_EVENT)){
			set_current_state(ButtonStates::PRESSED);
			for(const auto& localcall : callbacks_press)
				if(const auto& pval = std::get_if<curan::ui::Press>(&sig))
            		localcall(this,*pval,config);		
			return true;
		}
		
		if(interpreter.check(INSIDE_FIXED_AREA | MOUSE_CLICKED_LEFT)){
			set_current_state(ButtonStates::PRESSED);
			return true;
		}
		
		if(interpreter.check(INSIDE_FIXED_AREA)){
			set_current_state(ButtonStates::HOVER);
			return false;
		}
		
		if(interpreter.check(OUTSIDE_FIXED_AREA)){
			set_current_state(ButtonStates::WAITING);
			return false;
		}

		return false;
		};
		
	return lamb;
}

void Button::compile(){
	std::lock_guard<std::mutex> g{ get_mutex() };
	utilities::print<utilities::Severity::debug>("button{:d} : is compiled\n",(uintptr_t)this);


	auto text_font = SkFont(typeface, font_size, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	text_font.measureText(button_text.data(), button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(button_text.c_str(), text_font);

	if (system_icons.is_loaded() && icon_identifier.size()>0) {
		auto image = system_icons.get_icon(icon_identifier);
		if(image) icon_data = (*image).image;
	}
	compiled = true;
}

}
}