#ifndef CURAN_IMAGE_DISPLAY_HEADER_FILE_
#define CURAN_IMAGE_DISPLAY_HEADER_FILE_

#include <functional>
#include "definitions/UIdefinitions.h"
#include "Drawable.h"
#include "utils/Lockable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"

namespace curan {
	namespace ui {

		using image_provider = std::function<SkPixmap*()>;

		class ImageDisplay : public  Drawable, utils::Lockable<ImageDisplay>, utils::Connectable<ImageDisplay> {
			int width;
			int height;
			sk_sp<SkSurface> image_display_surface;

		public:
			struct Info {
				int width;
				int height;
			};

			ImageDisplay(Info& info);
			static std::shared_ptr<ImageDisplay> make(Info& info);
			void update_image(image_provider provider);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

		};
	}
}

#endif