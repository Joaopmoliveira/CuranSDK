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

		class Button;
		struct ConfigDraw;
		using buttoncallback = std::function<void(Button*, ConfigDraw*)>;

		class Button : public  Drawable , utilities::Lockable<Button> {
		public:

		enum class ButtonStates {
				WAITING,
				PRESSED,
				HOVER,
		};
		
		private:
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkColor text_color;
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			sk_sp<SkTypeface> typeface;
			size_t font_size = 15;
			std::string button_text;
			std::string icon_identifier;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			ButtonStates current_state = ButtonStates::WAITING;
			std::optional<buttoncallback> callback;
			IconResources& system_icons;
			bool compiled = false;

			Button(const std::string& button_text,IconResources& system_icons);

		public:

			static std::unique_ptr<Button> make(const std::string& button_text,IconResources& system_icons);

			void compile() override;

			~Button();

			drawablefunction draw() override;
			callablefunction call() override;

			inline Button& set_font_size(const size_t & in_size) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				font_size = in_size;
                return *(this);
			}

			inline Button& set_font_source(sk_sp<SkTypeface> in_typeface) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				typeface = in_typeface;
                return *(this);
			}

			inline Button& set_callback(buttoncallback in) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				callback = in;
                return *(this);
			}

			inline SkColor get_hover_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return hover_color;
			}

			inline Button& set_hover_color(SkColor color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				hover_color = color;
                return *(this);
			}

			inline SkColor get_waiting_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return waiting_color;
			}

			inline Button& set_waiting_color(SkColor new_waiting_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				waiting_color = new_waiting_color;
                return *(this);
			}

			inline Button& set_click_color(SkColor new_click_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				click_color = new_click_color;
                return *(this);
			}

			inline SkColor get_click_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return click_color;
			}

			inline ButtonStates get_current_state() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_state;
			}

			inline Button& set_current_state(ButtonStates state) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_state = state;
                return *(this);
			}

		};
	}
}

#endif