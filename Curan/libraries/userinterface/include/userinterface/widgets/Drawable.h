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

			Drawable();
			virtual ~Drawable() = default;

			inline Drawable& set_position(SkRect pos) {
				widget_rect = pos;
				return *(this);
			}

			inline SkRect get_position() {
				return widget_rect;
			}

			inline SkRect get_size() {
				return size;
			}

			inline void set_size(const SkRect& insize){
				size = insize;
			}

			inline bool interacts(double x, double y) {
				SkRect drawable;
				if (size.width() < 0.01f || size.height() < 0.01f)
					drawable = widget_rect;
				else
					drawable = size;
				drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
				return (drawable.fLeft < x && drawable.fRight > x && drawable.fTop < y && drawable.fBottom > y) ? true : false;
			}

			virtual void framebuffer_resize(const SkRect& new_page_size);
			virtual bool is_leaf();
			virtual void compile() = 0;
			virtual drawablefunction draw() = 0;
			virtual callablefunction call() = 0;
			virtual SkRect minimum_size();

		};
	}
}

#endif