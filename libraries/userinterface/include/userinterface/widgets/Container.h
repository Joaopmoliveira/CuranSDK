#ifndef CURAN_CONTAINER_HEADER_FILE_
#define CURAN_CONTAINER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Lockable.h"
#include <vector>
#include <memory>

namespace curan {
	namespace ui {

		class Container : public Drawable , utils::Lockable<Container>{
			SkPaint paint_layout;
			std::vector<std::shared_ptr<Drawable>> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;
			bool horizontaly_fixed = false;
			bool vertically_fixed = false;
		public:
			struct InfoLinearContainer {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Drawable>> layouts;
				std::vector<double> divisions;
				SkPaint paint_layout;
			};

			struct InfoVariableContainer {
				std::vector<std::shared_ptr<Drawable>> layouts;
				std::vector<SkRect> rectangles_of_contained_layouts;
				SkPaint paint_layout;
			};

			struct InfoPartiallyFixedContainer {
				std::vector<SkRect> rectangles_of_contained_layouts;
				std::vector<std::shared_ptr<Drawable>> layouts;
				bool horizontaly_fixed = false;
				bool vertically_fixed = false;
			};

			Container(InfoLinearContainer& info);
			Container(InfoVariableContainer& info);

			static std::shared_ptr<Container> make(InfoLinearContainer& info);
			static std::shared_ptr<Container> make(InfoVariableContainer& info);

			drawablefunction draw() override;
			callablefunction call() override;
			bool is_leaf() override;
			void framebuffer_resize() override;
			void linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal);

			inline std::vector<SkRect>& get_positioning() {
				return rectangles_of_contained_layouts;
			};
		};
	}
}
#endif