#ifndef CURAN_TEXTBLOB_HEADER_FILE_
#define CURAN_TEXTBLOB_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"

namespace curan {
	namespace ui {
		class TextBlob : public  Drawable, public utilities::Lockable, public SignalProcessor<TextBlob>{
			SkPaint paint;
			SkColor text_color;
			SkColor background_color;
			SkPaint paint_text;
			SkRect widget_rect_text;
			sk_sp<SkTypeface> typeface;
			size_t font_size = 15;
			sk_sp<SkTextBlob> text;
			std::string text_to_compile;
			bool compiled = false;
		public:
			TextBlob(const std::string& button_text);
			static std::unique_ptr<TextBlob> make(const std::string& button_text);
			drawablefunction draw() override;
			callablefunction call() override;

			inline TextBlob& set_font_size(size_t in_font_size){
				std::lock_guard<std::mutex> g{ get_mutex() };
				font_size = in_font_size;
				return *(this);
			}

			inline TextBlob& set_font_source(sk_sp<SkTypeface> intypeface){
				std::lock_guard<std::mutex> g{ get_mutex() };
				typeface = intypeface;
				return *(this);
			}

			inline TextBlob& set_text_color(SkColor in_text_color){
				std::lock_guard<std::mutex> g{ get_mutex() };
				text_color = in_text_color;
				return *(this);
			}

			inline TextBlob& set_background_color(SkColor in_background_color){
				std::lock_guard<std::mutex> g{ get_mutex() };
				background_color = in_background_color;
				return *(this);
			}

			void update(const std::string& user_text);
			
			void compile() override;
		};
	}
}

#endif