#ifndef CURAN_BUTTON_HEADER_FILE_
#define CURAN_BUTTON_HEADER_FILE_

#include "Drawable.h"
#include "Lockable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"
#include <optional>

namespace curan {
	namespace ui {

		class Button;

		using buttoncallback = std::function<void(std::shared_ptr<Button> val)>;

		class Button : public  Drawable , Lockable<Button>, utils::Connectable<Button> {
		public:
			struct Info {
				std::optional<buttoncallback> callback;
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

		enum class ButtonStates {
				WAITING,
				PRESSED,
				HOVER,
		};

		private:
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
			Button(Info& info);
			static std::shared_ptr<Button> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;
		};
	}
}

#endif