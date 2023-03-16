#ifndef CURAN_DRAWABLE_HEADER_FILE_
#define CURAN_DRAWABLE_HEADER_FILE_

#include "definitions/UIdefinitions.h"
#include <functional>
#include "Signal.h"

namespace curan {
	namespace ui {

		using drawablefunction = std::function<void(SkCanvas* canvas)>;
		using callablefunction = std::function<void(Signal sig)>;

		template<typename Derived>
		class Drawable {

		public:
			drawablefunction draw() {
				return static_cast<Derived*>(this)->impldraw();
			}

			callablefunction call() {
				return static_cast<Derived*>(this)->implcall();
			}
		};
	}
}

#endif