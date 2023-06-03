#ifndef CURAN_SLIDER_HEADER_FILE_
#define CURAN_SLIDER_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"

namespace curan {
	namespace ui {

		class Slider;

		using slidercallback = std::function<void(Slider*, ConfigDraw*)>;

		class Slider : public  Drawable, utilities::Lockable<Slider>, utilities::Connectable<Slider> {
		public:
			struct Info {
				std::optional<slidercallback> callback;
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkPaint paintButton;
				SkColor sliderColor;
				SkRect size;
				std::array<float, 2> limits = { 0.0f,100.0f };
				float current_value;
				float dragable_percent_size = 0.1f;
			};

			enum class SliderStates {
				WAITING,
				PRESSED,
				HOVER,
			};

		private:
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkColor slider_color;
			SkPaint paint;
			SkRect widget_rect_text;
			SliderStates current_state = SliderStates::WAITING;
			std::optional<slidercallback> callback;
			std::array<float, 2> limits = { 0.0f,100.0f };
			float current_value = 0.5;
			float value_pressed = 0.5;
			float dragable_percent_size = 0.1f;;

		public:
			Slider(Info& info);
			static std::shared_ptr<Slider> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

			inline void trigger(float in_current_value) {
				value_pressed = in_current_value;
			}

			inline float read_trigger() {
				return value_pressed;
			}

			inline void set_current_value(float in_current_value) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_value = in_current_value;
			}

			inline float get_current_value() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_value;
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

			inline SkColor get_click_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return click_color;
			}

			inline void set_click_color(SkColor new_click_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				click_color = new_click_color;
			}

			inline SliderStates get_current_state() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_state;
			}

			inline void set_current_state(SliderStates state) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_state = state;
			}

		};
	}
}

#endif