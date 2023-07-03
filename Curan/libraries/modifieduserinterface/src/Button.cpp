#include "modifieduserinterface/widgets/Button.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

Button::Button(const std::string& button_text,IconResources* system_icons) : system_icons{system_icons}{

}

Button::Button(Button&& other) : 
hover_color{std::move(other.hover_color)},
waiting_color{std::move(other.waiting_color)},
click_color{std::move(other.click_color)},
paint{std::move(other.paint)},
paint_text{std::move(other.paint_text)},
widget_rect_text{std::move(other.widget_rect_text)},
text_font{std::move(other.text_font)},
text{std::move(other.text)},
icon_data{std::move(other.icon_data)},
current_state{std::move(other.current_state)},
callback{std::move(other.callback)},
system_icons{std::move(other.system_icons)}
	{

}

Button::~Button(){

}

drawablefunction Button::draw(){
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
			float image_width = icon_data->width();
			float image_height = icon_data->height();
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

void Button::framebuffer_resize(){

}

}
}