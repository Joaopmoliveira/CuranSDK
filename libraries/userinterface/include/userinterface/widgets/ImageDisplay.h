#ifndef CURAN_IMAGE_DISPLAY_HEADER_FILE_
#define CURAN_IMAGE_DISPLAY_HEADER_FILE_

#include <functional>
#include "definitions/UIdefinitions.h"
#include "Drawable.h"
#include "utils/Lockable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Cancelable.h"
#include <list>

namespace curan {
	namespace ui {

		using image_provider = std::function<void(SkPixmap&)>;
		using custom_step = std::function<void(SkCanvas*, SkRect)>;
		using skia_image_producer = std::function<sk_sp<SkImage>(void)>;

		class ImageDisplay : public  Drawable, utilities::Lockable<ImageDisplay>, utilities::Connectable<ImageDisplay> {
			int width = 100;
			int height = 100;
			
			std::optional<skia_image_producer> old_image = std::nullopt;
			std::optional<skia_image_producer> images_to_render = std::nullopt;
			std::optional<custom_step> custom_drawing_call = std::nullopt;

		public:
			struct Info {
				int width;
				int height;
				std::optional<custom_step> custom_drawing_call;
			};

			ImageDisplay(Info& info);
			static std::shared_ptr<ImageDisplay> make(Info& info);
			void update_image(image_provider provider);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

			std::optional<skia_image_producer> get_image_wrapper();

			void override_image_wrapper(std::optional<skia_image_producer> wrapper);

			void update_custom_drawingcall(custom_step call);

			void clear_custom_drawingcall();

			std::optional<custom_step> get_custom_drawingcall();

			void update_batch(custom_step call, image_provider wrapper);
		};
	}
}

#endif