#ifndef CURAN_RADIO_BUTTON_HEADER_FILE_
#define CURAN_RADIO_BUTTON_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include <string>
#include <vector>
#include "SignalProcessor.h"

namespace curan {
	namespace ui {

		class RadioButton final : public  Drawable, public utilities::Lockable, public SignalProcessor<RadioButton> {
		public:
			enum class Arrangement {
				VERTICAL,
				HORIZONTAL
			};

			struct RadioItem {
				SkRect normalized_position;
				SkRect item_position;
				sk_sp<SkTextBlob> text;
				bool is_selected = false;
			};

			struct Info {
				Arrangement arrangement = Arrangement::VERTICAL;
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkPaint paintButton;
				SkPaint paintText;
				SkFont text_font;
				SkRect size;
				bool is_exclusive = false;
				std::vector<std::string> options;

				Info();

				bool validate();
			};

			enum class RadioButtonStates {
				WAITING,
				PRESSED,
				HOVER,
			};


		private:
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkPaint paint_button;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			RadioButtonStates current_state = RadioButtonStates::WAITING;
			std::vector<RadioItem> radio_items;
			bool is_exclusive = false;

		public:
			RadioButton(Info& info);
			static std::shared_ptr<RadioButton> make(Info& info);
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

			inline RadioButtonStates get_current_state() {
				std::lock_guard<std::mutex> g{ get_mutex() };
				return current_state;
			}

			inline void set_current_state(RadioButtonStates state) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				current_state = state;
			}
		};
	}
}


#endif