#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Button.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {

Button::Info::Info(IconResources& in_system_icons) : system_icons{ in_system_icons } {
	
}
		
Button::Button(Info& info) : Drawable{ info.size }, system_icons{ info.system_icons } {
	hover_color = info.hover_color;
	waiting_color = info.waiting_color;
	click_color = info.click_color;
	paint = info.paintButton;
	paint_text = info.paintText;
	text_font = info.textFont;
	callback = info.callback;
	text_font.measureText(info.button_text.data(), info.button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(info.button_text.c_str(), text_font);

	if (system_icons.is_loaded()) {
		sk_sp<SkImage> image;
		system_icons.get_icon(image, info.icon_identifier);
		icon_data = image;
	}
}

std::shared_ptr<Button> Button::make(Info& info) {
	return std::make_shared<Button>(info);
}

drawablefunction Button::draw() {
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
		drawable.offsetTo(widget_rect.centerX()- drawable.width()/2.0, widget_rect.centerY()- drawable.height() / 2.0);

		float text_offset_x = drawable.centerX() - widget_rect_text.width() / 2.0f;
		float text_offset_y = drawable.centerY() + widget_rect_text.height() / 2.0f;
				
		canvas->drawRect(drawable, paint);
		canvas->drawTextBlob(text, text_offset_x, text_offset_y, paint_text);

		if (icon_data.get() != nullptr) {
			float image_width = icon_data->width();
			float image_height = icon_data->height();
			float current_selected_width = drawable.width();
			float current_selected_height = drawable.height() - widget_rect_text.height();

			float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);

			float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + drawable.x();
			float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + drawable.y();

			SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
			canvas->drawImageRect(icon_data, current_selected_image_rectangle, opt);
		}
	};
	return lamb;
}

callablefunction Button::call() {
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
					if (callback) {
						auto val = *callback;
						val(this, config);
					}
					
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

void Button::framebuffer_resize() {

}

}
}