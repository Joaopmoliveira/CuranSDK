#ifndef CURAN_LIGHT_PAGE_HEADER_FILE_
#define CURAN_LIGHT_PAGE_HEADER_FILE_

#include "Container.h"

namespace curan {
	namespace ui {

		using post_signal_callback = std::function<void(Signal sig, bool page_interaction, ConfigDraw* config)>;

		struct compilation_results {
			std::vector<drawablefunction> callable_draw;
			std::vector<callablefunction> callable_signal;
		};

		class LightWeightPage {
			Container scene;
			std::atomic<bool> is_dirty = false;
			compilation_results compiled_scene;
			SkColor backgroundcolor = SK_ColorWHITE;
			post_signal_callback post_signal_processing;

        public:
			LightWeightPage(Container&& contained, SkColor backgroundcolor);
			LightWeightPage(post_signal_callback post_sig,Container&& contained, SkColor backgroundcolor);
			LightWeightPage(const LightWeightPage& other);
			LightWeightPage& operator=(const LightWeightPage& other);
			LightWeightPage(LightWeightPage&& other);
			~LightWeightPage();

			LightWeightPage& draw(SkCanvas* canvas);

			bool propagate_signal(Signal sig, ConfigDraw* config);

			LightWeightPage& propagate_size_change(SkRect& new_size);

			LightWeightPage& set_post_signal(post_signal_callback call);

			inline LightWeightPage& set_dirtyness(bool var) {
				is_dirty = var;
                return *(this);
			}
		};

	}
}

#endif