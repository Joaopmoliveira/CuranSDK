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
		public:

        enum class ContainerType{
            LINEAR_CONTAINER,
            VARIABLE_CONTAINER
        };

        enum class Arrangement {
			VERTICAL,
			HORIZONTAL
		};

		~Container();

		drawablefunction draw();
		callablefunction call();

		inline Container& set_color(SkColor color){
			std::lock_guard<std::mutex> g{ get_mutex() };
			layout_color = color;
			return *(this);
		}

		inline bool is_leaf(){
			return false;
		}

		Container& framebuffer_resize();
		Container& linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal);

		inline std::vector<SkRect>& get_positioning() {
			return rectangles_of_contained_layouts;
		};

		Container& operator<<(Widget&& widget);

		static std::unique_ptr<Container> make(const ContainerType& type, const Arrangement& arragement);

		private:

			Container(const ContainerType& type,const Arrangement& arragement);

			SkPaint paint_layout;
			std::vector<Widget> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;
			bool horizontaly_fixed = false;
			bool vertically_fixed = false;
			ContainerType type;
			Arrangement arragement;
			SkColor layout_color;
			void compile();
		};
	}
}
#endif