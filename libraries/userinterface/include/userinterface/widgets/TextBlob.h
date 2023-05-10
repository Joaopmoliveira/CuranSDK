#ifndef CURAN_BUTTON_HEADER_FILE_
#define CURAN_BUTTON_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"

namespace curan {
	namespace ui {
		class TextBlob : public  Drawable, utils::Lockable<TextBlob>, utils::Connectable<TextBlob> {
		public:
			struct Info {
				SkPaint paint;
				SkPaint paintText;
				SkFont textFont;
				SkRect size;
				std::string button_text;
			};

		private:
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkFont text_font;
			sk_sp<SkTextBlob> text;

		public:
			TextBlob(Info& info);
			static std::shared_ptr<TextBlob> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;
		};
	}
}

#endif