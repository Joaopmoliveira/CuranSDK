#ifndef CURAN_CONTAINER_HEADER_FILE_
#define CURAN_CONTAINER_HEADER_FILE_

#include "Drawable.h"
#include "Lockable.h"
#include <vector>
#include <memory>

namespace curan {
	namespace ui {
		class Container : public Drawable {
		public:
			SkPaint paint_layout;
			std::vector<std::shared_ptr<Drawable>> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;
		};
	}
}
#endif