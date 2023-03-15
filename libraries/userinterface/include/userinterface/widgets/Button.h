#ifndef CURAN_BUTTON_HEADER_FILE_
#define CURAN_BUTTON_HEADER_FILE_

#include "Drawable.h"

namespace curan {
	namespace ui {
		class Button : Drawable<Button> {
			enum class ButtonStates {
				WAITING,
				PRESSED,
				HOVER,
			};

		protected:
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkRect size;
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			ButtonStates current_state = ButtonStates::WAITING;

		public:
			struct Info {
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkPaint paintButton;
				SkPaint paintText;
				SkFont textFont;
				SkRect size;
				std::string button_text;
				std::string icon_identifier;
			};

			Button(Info& info);
			static std::shared_ptr<Button> make(Info& info);
			drawablefunction impldraw();
			callablefunction implcall()();
		};
	}
}

#endif