#ifndef CURAN_CONTAINER_HEADER_FILE_
#define CURAN_CONTAINER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Lockable.h"
#include <vector>
#include <memory>
#include "Widget.h"

namespace curan {
	namespace ui {

		class Container : public Drawable , utilities::Lockable<Container>{
			SkPaint paint_layout;
			std::vector<Widget> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;
			bool horizontaly_fixed = false;
			bool vertically_fixed = false;

		public:

        enum class ContainerType{
            LINEAR_CONTAINER,
            VARIABLE_CONTAINER
        };

        enum class Arrangement {
			VERTICAL,
			HORIZONTAL
		};

			explicit Container(ContainerType type, Arrangement arragement);
			Container(Container&& container);
			
			drawablefunction draw();
			callablefunction call();
			bool is_leaf();
			Container& framebuffer_resize();
			Container& linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal);

			inline std::vector<SkRect>& get_positioning() {
				return rectangles_of_contained_layouts;
			};
		};
	}
}
#endif