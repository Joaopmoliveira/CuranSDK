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

		using buttoncallback = std::function<void()>;

		class Slider : public  Drawable, utils::Lockable<Slider>, utils::Connectable<Slider> {
		public:
			struct Info {
				std::optional<buttoncallback> callback;
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkPaint paintButton;
				SkPaint slicerColor;
				SkPaint paintText;
				SkFont textFont;
				SkRect size;
				std::array<double, 2> limits = { 0.0,100.0 };

				Info(IconResources& in_system_icons);
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
			SkPaint paint_text;
			SkRect widget_rect_text;
			SliderStates current_state = SliderStates::WAITING;
			std::optional<buttoncallback> callback;
			std::array<double, 2> limits = { 0.0,100.0 };

		public:
			Slider(Info& info);
			static std::shared_ptr<Slider> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

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