#ifndef CURAN_DRAWABLE_HEADER_FILE_
#define CURAN_DRAWABLE_HEADER_FILE_

#include "UIdefinitions.h"

namespace curan {
	namespace ui {
		template<typename Derived>
		class Drawable {
			void draw(SkCanvas* canvas) { 
				(static_cast<Derived*>(this))->draw(canvas);
			}
		};
	}
}

#endif