#ifndef CURAN_DRAWABLE_HEADER_FILE_
#define CURAN_DRAWABLE_HEADER_FILE_

#include "definitions/UIdefinitions.h"
#include <functional>
#include "Signal.h"

namespace curan {
	namespace ui {

		using drawablefunction = std::function<void(SkCanvas* canvas)>;
		using callablefunction = std::function<void(Signal sig)>;

		enum Arrangement {
			VERTICAL,

			HORIZONTAL,

			VARIABLE
		};

		class Drawable {
			SkRect widget_rect = SkRect::MakeLTRB(10,10,100,100);
		public:
			drawablefunction virtual draw() = 0;

			callablefunction virtual call() = 0;

			/*
			
			*/
			void virtual framebuffer_resize(SkRect& pos) = 0;

			inline void set_position(SkRect pos) {
				widget_rect = pos;
			}

			inline SkRect get_position() {
				return widget_rect;
			}

			inline bool interacts(double x, double y) {
				return (widget_rect.fLeft < x && widget_rect.fRight > x && widget_rect.fTop < y && widget_rect.fBottom > y) ? true : false;
			}
		};
	}
}

#endif