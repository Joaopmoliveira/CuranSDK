#ifndef CURAN_TEXTBLOB_HEADER_FILE_
#define CURAN_TEXTBLOB_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"

namespace curan {
	namespace ui {
		class TextBlob : public  Drawable, utilities::Lockable<TextBlob>{
			SkPaint paint;
			SkColor text_color;
			SkColor background_color;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			std::string text_to_compile;
		public:
			TextBlob(const std::string& button_text);
			static std::unique_ptr<TextBlob> make(const std::string& button_text);
			drawablefunction draw() override;
			callablefunction call() override;

			TextBlob& set_text_color(SkColor in_text_color){
				std::lock_guard<std::mutex> g{ get_mutex() };
				text_color = in_text_color;
			}

			TextBlob& set_background_color(SkColor in_background_color){
				std::lock_guard<std::mutex> g{ get_mutex() };
				background_color = in_background_color;
			}

			void framebuffer_resize() override;
			void compile() override;
		};
	}
}

#endif