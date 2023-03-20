#ifndef CURAN_CONTAINERLINEAR_HEADER_FILE_
#define CURAN_CONTAINERLINEAR_HEADER_FILE_

#include "Container.h"
#include "Lockable.h"
#include <vector>
#include <memory>

namespace curan {
	namespace ui {
		class ContainerLinear : public Container, Lockable<ContainerLinear> {
		public:
			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Drawable>> layouts;
				std::vector<double> divisions;
				SkPaint paint_layout;
			};

			ContainerLinear(Info& info);
			static std::shared_ptr<ContainerLinear> make(Info& info);
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