#ifndef CURAN_CONTAINER_HEADER_FILE_
#define CURAN_CONTAINER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Lockable.h"
#include <vector>
#include <memory>

namespace curan {
	namespace ui {

		class Container : public Drawable , utilities::Lockable<Container>{
			SkPaint paint_layout;
			std::vector<std::shared_ptr<Drawable>> contained_layouts;
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

			drawablefunction draw() override;
			callablefunction call() override;
			bool is_leaf() override;
			Container& framebuffer_resize() override;
			Container& linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal);

			inline std::vector<SkRect>& get_positioning() {
				return rectangles_of_contained_layouts;
			};
		};
	}
}
#endif