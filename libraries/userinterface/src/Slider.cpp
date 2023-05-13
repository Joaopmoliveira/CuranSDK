#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Slider.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {


Slider::Slider(Info& info) : Drawable{ info.size } {
    hover_color = info.hover_color;
    waiting_color = info.waiting_color;
    click_color = info.click_color;
    paint = info.paintButton;
    callback = info.callback;
    slider_color = info.sliderColor;
    limits = info.limits;
    dragable_percent_size = info.dragable_percent_size;
}

std::shared_ptr<Slider> Slider::make(Info& info) {
    return std::make_shared<Slider>(info);
}

drawablefunction Slider::draw() {
    auto lamb = [this](SkCanvas* canvas) {
        std::lock_guard<std::mutex> g{ get_mutex() };

        auto widget_rect = get_position();
        auto size = get_size();

        SkRect drawable = size;
        drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);

        paint.setColor(slider_color);
		canvas->drawRoundRect(drawable, drawable.height() / 2.0, drawable.height() / 2.0, paint);

        switch (current_state) {
        case SliderStates::WAITING:
            paint.setColor(waiting_color);
            break;
        case SliderStates::HOVER:
            paint.setColor(hover_color);
            break;
        case SliderStates::PRESSED:
            paint.setColor(click_color);
            break;
        }

        SkRect dragable = SkRect::MakeXYWH(drawable.x()+ drawable.width() * current_value, drawable.y(), drawable.width() * dragable_percent_size, drawable.height());
		canvas->drawRoundRect(dragable, drawable.height() / 2.0, drawable.height() / 2.0,paint);
    };
    return lamb;
}

callablefunction Slider::call() {
	auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utils::overloaded{
			[this](Empty arg) {

			},
			[this,&interacted](Move arg) {
				static Move previous_arg = arg;
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos)) {
					if (previous_state != SliderStates::PRESSED)
						current_state_local = SliderStates::HOVER;
					else {
						auto widget_rect = get_position();
						auto size = get_size();
						SkRect drawable = size;
						drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);
						auto offset_x = (arg.xpos - drawable.x()) / size.width();
						auto current_val = get_current_value();
						current_val += offset_x-read_trigger();
						trigger(offset_x);
						if (current_val < 0.0) current_val = 0.0;
						if (current_val > 1.0) current_val = 1.0;
						set_current_value(current_val);
					}
				}
				else {
					current_state_local = SliderStates::WAITING;
				}
				if (previous_state != current_state_local)
					interacted = true;
				previous_arg = arg;
				set_current_state(current_state_local);
			},
			[this,&interacted](Press arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos,arg.ypos)) {
					auto widget_rect = get_position();
					auto size = get_size();
					SkRect drawable = size;
					drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);
					trigger((arg.xpos - drawable.x()) / size.width());
					current_state_local = SliderStates::PRESSED;
					if (callback) {
						auto val = *callback;
						val();
					}

				}
				else
					current_state_local = SliderStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this](Scroll arg) {;

			},
			[this,&interacted](Unpress arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos))
					current_state_local = SliderStates::HOVER;
				else
					current_state_local = SliderStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
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

void Slider::framebuffer_resize() {

}

}
}