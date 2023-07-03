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

		class Button : public  Drawable , utilities::Lockable<Button>{
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
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			ButtonStates current_state = ButtonStates::WAITING;
			std::optional<buttoncallback> callback;
			IconResources* system_icons = nullptr;

			void compile();

		public:

            Button(const std::string& button_text,IconResources* system_icons = nullptr);

			Button(const Button& other) = delete;
			Button& operator=(const Button& other)= delete;

			Button(Button&& other);
			~Button();

			drawablefunction draw();
			callablefunction call();
			void framebuffer_resize();

			inline Button& set_callback(buttoncallback in) {
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