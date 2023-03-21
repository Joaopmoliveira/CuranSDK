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
			std::shared_ptr<Container> scene;
			std::atomic<bool> is_dirty = false;
			compilation_results compiled_scene;

			Page(std::shared_ptr<Container> drawables);

		public:

			static std::shared_ptr<Page> make(std::shared_ptr<Container> drawables);

			void draw(SkCanvas* canvas);

			void propagate_signal(Signal sig);

			void propagate_size_change(SkRect& new_size);
		};
	}
}

#endif