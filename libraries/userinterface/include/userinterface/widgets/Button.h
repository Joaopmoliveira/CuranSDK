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

		class Button : public  Drawable , utilities::Lockable<Button>, utilities::Connectable<Button> {
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
				IconResources& system_icons;

				Info(IconResources& in_system_icons);
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
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			ButtonStates current_state = ButtonStates::WAITING;
			std::optional<buttoncallback> callback;
			IconResources& system_icons;

		public:
			Button(Info& info);
			static std::shared_ptr<Button> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

			inline void set_callback(buttoncallback in) {
				callback = in;
			}

			inline SkColor get_hover_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return hover_color;
			}

			inline void set_hover_color(SkColor color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				hover_color = color;
			}

			inline SkColor get_waiting_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return waiting_color;
			}

			inline void set_waiting_color(SkColor new_waiting_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				waiting_color = new_waiting_color;
			}

			inline void set_click_color(SkColor new_click_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				click_color = new_click_color;
			}

			inline SkColor get_click_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return click_color;
			}

			inline ButtonStates get_current_state() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_state;
			}

			inline void set_current_state(ButtonStates state) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_state = state;
			}

		};
	}
}

#endif