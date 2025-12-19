#ifndef CURAN_SLIDER_HEADER_FILE_
#define CURAN_SLIDER_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"

namespace curan {
	namespace ui {

		class Slider;

		using slidercallback = std::function<void(Slider*, ConfigDraw*)>;

		class Slider final : public  Drawable, public utilities::Lockable, public SignalProcessor<Slider>{
		public:
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
			std::pair<double,double> old_pressed_value;

			SkPaint paint;
			SkRect widget_rect_text;
			SliderStates current_state = SliderStates::WAITING;
			std::optional<slidercallback> callback;
			std::array<float, 2> limits = { 0.0f,100.0f };
			float current_value = 0.5;
			float value_pressed = 0.5;
			float dragable_percent_size = 0.1f;
			SignalInterpreter interpreter;

			Slider(const std::array<float,2>& in_limits);

		public:

			static std::unique_ptr<Slider> make(const std::array<float,2>& in_limits);

			~Slider();

			drawablefunction draw() override;
			callablefunction call() override;
			SkRect minimum_size() override;

			void compile() override;

			inline Slider& set_callback(slidercallback in_callback){
				std::lock_guard<std::mutex> g{ get_mutex() };
				callback = in_callback;
				return *(this);
			}

			inline std::array<float, 2> get_limits(){
				return limits;
			}

			inline Slider& trigger(float in_current_value) {
				value_pressed = in_current_value;
				return *(this);
			}

			inline float read_trigger() {
				return value_pressed;
			}

			inline Slider& set_current_value(float in_current_value) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				if (in_current_value < 0.0) in_current_value = 0.0;
				if (in_current_value > 1.0) in_current_value = 1.0;
				current_value = in_current_value;
				return *(this);
			}

			inline float get_current_value() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_value;
			}

			inline SkColor get_hover_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return hover_color;
			}

			inline Slider& set_hover_color(SkColor color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				hover_color = color;
				return *(this);
			}

			inline SkColor get_waiting_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return waiting_color;
			}

			inline Slider& set_waiting_color(SkColor new_waiting_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				waiting_color = new_waiting_color;
				return *(this);
			}

			inline SkColor get_click_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return click_color;
			}

			inline Slider& set_click_color(SkColor new_click_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				click_color = new_click_color;
				return *(this);
			}

			inline SkColor get_slider_color() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return slider_color;
			}

			inline Slider& set_slider_color(SkColor new_slider_color) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				slider_color = new_slider_color;
				return *(this);
			}			

			inline SliderStates get_current_state() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_state;
			}

			inline Slider& set_current_state(SliderStates state) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_state = state;
				return *(this);
			}
		};
	}
}

#endif