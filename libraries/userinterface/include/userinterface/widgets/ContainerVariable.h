#ifndef CURAN_CONTAINERVARIABLE_HEADER_FILE_
#define CURAN_CONTAINERVARIABLE_HEADER_FILE_

#include "Container.h"
#include "Lockable.h"
#include <vector>
#include <memory>

namespace curan {
	namespace ui {
		class ContainerVariable : public Container, Lockable<ContainerVariable> {
		public:

			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Drawable>> layouts;
				std::vector<SkRect> rectangles_of_contained_layouts;
				SkPaint paint_layout;
			};

			ContainerVariable(Info& info);
			static std::shared_ptr<ContainerVariable> make(Info& info);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;
		private:
			SkPaint paint_layout;
			Arrangement arrangment;
		};
	}
}
#endif