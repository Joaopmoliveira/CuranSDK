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
		public:
			struct Info {
				post_signal_callback post_sig;
				std::shared_ptr<Container> contained;
				SkColor backgroundcolor = SK_ColorWHITE;

				Info();
			};

		private:

			std::shared_ptr<Container> scene;
			std::atomic<bool> is_dirty = false;
			compilation_results compiled_scene;
			SkColor backgroundcolor = SK_ColorWHITE;
			post_signal_callback post_signal_processing;
			LightWeightPage(Info drawables);

		public:
			static std::unique_ptr<LightWeightPage> make(Info);

			void draw(SkCanvas* canvas);

			bool propagate_signal(Signal sig, ConfigDraw* config);

			void propagate_size_change(SkRect& new_size);

			inline void set_dirtyness(bool var) {
				is_dirty = var;
			}
		};

	}
}

#endif