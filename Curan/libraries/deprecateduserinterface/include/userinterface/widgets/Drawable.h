#ifndef CURAN_DRAWABLE_HEADER_FILE_
#define CURAN_DRAWABLE_HEADER_FILE_

#include "definitions/UIdefinitions.h"
#include <functional>
#include "Signal.h"

namespace curan {
	namespace ui {

		struct ConfigDraw;

		using drawablefunction = std::function<void(SkCanvas* canvas)>;
		using callablefunction = std::function<bool(Signal sig, ConfigDraw*)>;

		enum class Arrangement {
			VERTICAL,
			HORIZONTAL,
		};

		class Drawable {
			SkRect widget_rect = SkRect::MakeLTRB(10,10,100,100);
			SkRect size = SkRect::MakeWH(100, 100);
		public:

			Drawable(SkRect size);

			drawablefunction virtual draw() = 0;

			callablefunction virtual call() = 0;

			void virtual framebuffer_resize() = 0;

			bool virtual is_leaf();

			inline void set_position(SkRect pos) {
				widget_rect = pos;
			}

			inline SkRect get_position() {
				return widget_rect;
			}

			inline SkRect get_size() {
				return size;
			}

			inline bool interacts(double x, double y) {
				SkRect drawable;
				if (size.width() < 0.01 || size.height() < 0.01)
					drawable = widget_rect;
				else
					drawable = size;
				drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0, widget_rect.centerY() - drawable.height() / 2.0);
				return (drawable.fLeft < x && drawable.fRight > x && drawable.fTop < y && drawable.fBottom > y) ? true : false;
			}
		};
	}
}

#endif