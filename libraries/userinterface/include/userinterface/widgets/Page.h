#ifndef CURAN_PAGE_HEADER_FILE_
#define CURAN_PAGE_HEADER_FILE_

#include <vector>
#include <atomic>
#include "Container.h"

namespace curan {
	namespace ui {

		struct compilation_results {
			std::vector<drawablefunction> callable_draw;
			std::vector<callablefunction> callable_signal;
		};

		class Page {
		public:
			struct Info {
				std::shared_ptr<Container> contained;
				SkColor backgroundcolor = SK_ColorWHITE;
			};

		private:	

			std::shared_ptr<Container> scene;
			std::atomic<bool> is_dirty = false;
			compilation_results compiled_scene;
			SkColor backgroundcolor = SK_ColorWHITE;
			Page(Info drawables);

		public:
			static std::shared_ptr<Page> make(Info);

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