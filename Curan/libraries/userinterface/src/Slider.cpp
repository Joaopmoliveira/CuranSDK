#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Slider.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
namespace ui {

Slider::Slider(const std::array<float,2>& in_limits){
    hover_color = SK_ColorCYAN;
    waiting_color = SK_ColorDKGRAY;
    click_color = SK_ColorLTGRAY;

    slider_color = SK_ColorGRAY;
    limits = in_limits;

	paint.setStyle(SkPaint::kStrokeAndFill_Style);
	paint.setAntiAlias(true);
	paint.setStrokeWidth(20);
	paint.setColor(slider_color);
	paint.setStrokeJoin(SkPaint::kRound_Join);
	paint.setStrokeCap(SkPaint::kRound_Cap);
}

std::unique_ptr<Slider> Slider::make(const std::array<float,2>& in_limits) {
	std::unique_ptr<Slider> slider = std::unique_ptr<Slider>(new Slider(in_limits));
    return slider;
}

void Slider::compile(){
	
}

Slider::~Slider(){

}

drawablefunction Slider::draw() {
    auto lamb = [this](SkCanvas* canvas) {
        auto widget_rect = get_position();
        auto size = get_size();

        SkRect drawable = size;
        drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);

        paint.setColor(slider_color);
		canvas->drawRoundRect(drawable, drawable.height() / 2.0f, drawable.height() / 2.0f, paint);

        switch (current_state) {
        case SliderStates::WAITING:
            paint.setColor(get_waiting_color());
            break;
        case SliderStates::HOVER:
            paint.setColor(get_hover_color());
            break;
        case SliderStates::PRESSED:
            paint.setColor(get_click_color());
            break;
        }

        SkRect dragable = SkRect::MakeXYWH(drawable.x()+ (drawable.width()*(1-dragable_percent_size)) * current_value, drawable.y(), drawable.width() * dragable_percent_size, drawable.height());
		canvas->drawRoundRect(dragable, drawable.height() / 2.0f, drawable.height() / 2.0f,paint);
    };
    return lamb;
}

SkRect Slider::minimum_size(){
	auto size = get_size();
	return SkRect::MakeWH(size.width()+20,size.height()+20);
}

callablefunction Slider::call() {
	auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utilities::overloaded{
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
						drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
						auto offset_x = ((float)arg.xpos - drawable.x()) / size.width();
						auto current_val = get_current_value();
						current_val += offset_x-read_trigger();
						trigger(offset_x);
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
			[this,&interacted,config](Press arg) {
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
						val(this,config);
					}
				}
				else
					current_state_local = SliderStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this,&interacted,config](Scroll arg) {;
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos)) {
					auto widget_rect = get_position();
					auto size = get_size();
					SkRect drawable = size;
					drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
					auto offsetx = (float)arg.xoffset / size.width();
					auto offsety = (float)arg.yoffset / size.width();
					auto current_val = get_current_value();
					current_val += (std::abs(offsetx)>std::abs(offsety)) ? offsetx : offsety;
					set_current_value(current_val);
					current_state_local = SliderStates::PRESSED;
					if (callback) {
						auto val = *callback;
						val(this, config);
					}
				}
				else {
					current_state_local = SliderStates::WAITING;
				}
				if (previous_state != current_state_local)
					interacted = true;
				set_current_state(current_state_local);
			},
			[this,&interacted,config](Unpress arg) {
				auto previous_state = get_current_state();
				auto current_state_local = get_current_state();
				if (interacts(arg.xpos, arg.ypos))
					current_state_local = SliderStates::HOVER;
				else
					current_state_local = SliderStates::WAITING;
				if (previous_state != current_state_local)
					interacted = true;
				if (callback) {
					auto val = *callback;
					val(this, config);
				}
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

}
}