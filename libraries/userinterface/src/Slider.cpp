#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Slider.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>

namespace curan
{
	namespace ui
	{

		Slider::Slider(const std::array<float, 2> &in_limits)
		{
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

		std::unique_ptr<Slider> Slider::make(const std::array<float, 2> &in_limits)
		{
			std::unique_ptr<Slider> slider = std::unique_ptr<Slider>(new Slider(in_limits));
			return slider;
		}

		void Slider::compile()
		{
		}

		Slider::~Slider()
		{
		}

		drawablefunction Slider::draw()
		{
			auto lamb = [this](SkCanvas *canvas)
			{
				auto widget_rect = get_position();
				auto size = get_size();

				SkRect drawable = size;
				drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);

				paint.setColor(slider_color);
				canvas->drawRoundRect(drawable, drawable.height() / 2.0f, drawable.height() / 2.0f, paint);

				switch (current_state)
				{
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

				SkRect dragable = SkRect::MakeXYWH(drawable.x() + (drawable.width() * (1 - dragable_percent_size)) * current_value, drawable.y(), drawable.width() * dragable_percent_size, drawable.height());
				canvas->drawRoundRect(dragable, drawable.height() / 2.0f, drawable.height() / 2.0f, paint);
			};
			return lamb;
		}

		SkRect Slider::minimum_size()
		{
			auto size = get_size();
			return SkRect::MakeWH(size.width() + 20, size.height() + 20);
		}

		callablefunction Slider::call()
		{
			auto lamb = [this](Signal sig, ConfigDraw *config)
			{
				auto check_inside_fixed_area = [this](double x, double y)
				{
					auto widget_rect = get_position();
					auto size = get_size();
					SkRect drawable = size;
					drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
					return drawable.contains(x, y);
				};
				interpreter.process(check_inside_fixed_area, check_inside_fixed_area, sig);

				if (interpreter.check(INSIDE_FIXED_AREA | SCROLL_EVENT))
				{
					auto widget_rect = get_position();
					auto size = get_size();
					SkRect drawable = size;
					auto arg = std::get<curan::ui::Scroll>(sig);
					drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
					auto offsetx = (float)arg.xoffset / size.width();
					auto offsety = (float)arg.yoffset / size.width();
					auto current_val = get_current_value();
					current_val += (std::abs(offsetx) > std::abs(offsety)) ? offsetx : offsety;
					set_current_value(current_val);
					if (callback)
					{
						auto val = *callback;
						val(this, config);
					}
					set_current_state(SliderStates::PRESSED);
					return true;
				}

				if(interpreter.check(INSIDE_FIXED_AREA | MOUSE_CLICKED_LEFT_EVENT)){
					old_pressed_value = interpreter.last_move();
					return true;
				}

				if (interpreter.check(MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED))
				{
					auto widget_rect = get_position();
					auto size = get_size();
					SkRect drawable = size;
					auto [xarg,yarg] = interpreter.last_move();
					auto [xarg_last,yarg_last] = old_pressed_value;
					auto current_val = get_current_value();
					drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
					auto offsetx = (float)(xarg-xarg_last) / size.width();
					set_current_value(offsetx+current_val);
					set_current_state(SliderStates::PRESSED);
					if (callback)
					{
						auto val = *callback;
						val(this, config);
					}
					old_pressed_value = interpreter.last_move();
					return true;
				}

				if (interpreter.check(INSIDE_FIXED_AREA))
				{
					set_current_state(SliderStates::HOVER);
					return false;
				}

				if (interpreter.check(OUTSIDE_FIXED_AREA | MOUSE_MOVE_EVENT))
				{
					set_current_state(SliderStates::WAITING);
					return false;
				}

				set_current_state(SliderStates::WAITING);

				return false;
			};
			return lamb;
		}

	}
}