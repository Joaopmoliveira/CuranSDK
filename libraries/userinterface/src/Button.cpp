#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/SingletonIconResources.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {
		
Button::Button(Info& info) : Drawable{ info.size } {
	hover_color = info.hover_color;
	waiting_color = info.waiting_color;
	click_color = info.click_color;
	paint = info.paintButton;
	paint_text = info.paintText;
	text_font = info.textFont;
	text_font.measureText(info.button_text.data(), info.button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
	text = SkTextBlob::MakeFromString(info.button_text.c_str(), text_font);

	IconResources* resources = IconResources::Get();

	sk_sp<SkImage> image;
		resources->GetIcon(image, info.icon_identifier);
		icon_data = image;
}

std::shared_ptr<Button> Button::make(Info& info) {
	return std::make_shared<Button>(info);
}

drawablefunction Button::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		switch (current_state) {
		case ButtonStates::WAITING:
			paint.setColor(waiting_color);
			break;
		case ButtonStates::HOVER:
			paint.setColor(hover_color);
			break;
		case ButtonStates::PRESSED:
			paint.setColor(click_color);
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
	auto lamb = [this](Signal sig) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		bool interacted = false;
		std::visit(utils::overloaded{
			[this](Empty arg) {

			},
			[this,&interacted](Move arg) {
				auto previous_state = current_state;
				if (interacts(arg.xpos,arg.ypos))
					current_state = ButtonStates::HOVER;
				else
					current_state = ButtonStates::WAITING;
				if (previous_state != current_state)
					interacted = true;
			},
			[this,&interacted](Press arg) {
				auto previous_state = current_state;
				if (interacts(arg.xpos,arg.ypos)) {
					current_state = ButtonStates::PRESSED;
				}
				else
					current_state = ButtonStates::WAITING;
				if (previous_state != current_state)
					interacted = true;
			},
			[this](Scroll arg) {;

			},
			[this,&interacted](Unpress arg) {
				auto previous_state = current_state;
				if (interacts(arg.xpos, arg.ypos))
					current_state = ButtonStates::HOVER;
				else
					current_state = ButtonStates::WAITING;
				if (previous_state != current_state)
					interacted = true;
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