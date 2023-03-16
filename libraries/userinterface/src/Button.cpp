#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/SingletonIconResources.h"
#include "utils/Overloading.h"
#include <variant>

namespace curan {
	namespace ui {
		Button::Button(Info& info) {
			hover_color = info.hover_color;
			waiting_color = info.waiting_color;
			click_color = info.click_color;
			paint = info.paintButton;
			paint_text = info.paintText;
			size = info.size;
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

		bool Button::interacts(double x, double y) {
			return true;
		}

		drawablefunction Button::impldraw() {
			auto lamb = [this](SkCanvas* canvas) {
				LockGuard<Button> g{ this };
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
				float text_offset_x = widget_rect.centerX() - widget_rect_text.width() / 2.0f;
				float text_offset_y = widget_rect.centerY() + widget_rect_text.height() / 2.0f;
				float box_offset_x = widget_rect.centerX() - size.width() / 2.0f;
				float box_offset_y = widget_rect.centerY() - size.height() / 2.0f;
				SkRect current_area = SkRect::MakeXYWH(box_offset_x, box_offset_y, std::min(widget_rect.width(), size.width()), std::min(widget_rect.height(), size.height()));
				canvas->drawRect(current_area, paint);
				canvas->drawTextBlob(text, text_offset_x, current_area.fBottom, paint_text);

				if (icon_data.get() != nullptr) {
					float image_width = icon_data->width();
					float image_height = icon_data->height();

					float current_selected_width = current_area.width();
					float current_selected_height = current_area.height() - widget_rect_text.height();

					float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);

					float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + current_area.x();
					float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + current_area.y();

					SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

					SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
					canvas->drawImageRect(icon_data, current_selected_image_rectangle, opt);
				}
			};
			return lamb;
		}

		callablefunction Button::implcall() {
			auto lamb = [this](Signal sig) {
				LockGuard<Button> g{this};

				std::visit(utils::overloaded{
				[this](Empty arg) {;

					},
				[this](Move arg) {;
					if (interacts(arg.xpos,arg.ypos))
						current_state = ButtonStates::HOVER;
					},
				[this](Press arg) {;
					if (interacts(arg.xpos,arg.ypos)) {
						current_state = ButtonStates::PRESSED;
					}
					else
						current_state = ButtonStates::WAITING;
					},
				[this](Scroll arg) {;

					},
				[this](Unpress arg) {;
					current_state = ButtonStates::WAITING;
					},
				[this](ItemDropped arg) {;

					}},
				sig);
			};
			return lamb;
		}
	}
}