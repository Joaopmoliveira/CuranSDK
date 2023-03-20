#ifndef CURAN_PAGE_HEADER_FILE_
#define CURAN_PAGE_HEADER_FILE_

#include <list>
#include "Container.h"

namespace curan {
	namespace ui {

		struct compilation_results {
			std::vector<drawablefunction> callable_draw;
			std::vector<callablefunction> callable_signal;
		};

		compilation_results compile_drawables(Container& drawables) {
		
		}

		class Page {
			std::list<shared_ptr<Container>> contained_widgets;
			std::atomic<bool> is_dirty = false;
			compilation_results compiled_scene;

		public:
			struct Info {

			};

			Page(Info& info);

			void draw(SkCanvas* canvas) {
				if(is_dirty)
					for(auto& drawcall : compiled_scene.callable_draw)
						drawcall(canvas);
			}

			void propagate_signal(Signal sig) {
				for (auto& sigcall : compiled_scene.callable_signal) {
					if (sigcall(sig)) {
						is_dirty = true;
						break;
					}
				};
			}

		};
	}
}

#endif