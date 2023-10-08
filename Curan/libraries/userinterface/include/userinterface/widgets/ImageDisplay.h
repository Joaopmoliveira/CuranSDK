#ifndef CURAN_IMAGE_DISPLAY_HEADER_FILE_
#define CURAN_IMAGE_DISPLAY_HEADER_FILE_

#include <functional>
#include "definitions/UIdefinitions.h"
#include "Drawable.h"
#include "utils/Lockable.h"
#include <list>
#include "SignalProcessor.h"

namespace curan {
	namespace ui {

		using image_provider = std::function<void(SkPixmap&)>;
		using custom_step = std::function<void(SkCanvas*, SkRect)>;
		using skia_image_producer = std::function<sk_sp<SkImage>(void)>;

		class ImageDisplay : public  Drawable, utilities::Lockable<ImageDisplay>, SignalProcessor<ImageDisplay>{
			int width = -1;
			int height = -1;
			bool compiled = false;
			std::optional<skia_image_producer> old_image = std::nullopt;
			std::optional<skia_image_producer> images_to_render = std::nullopt;
			std::optional<custom_step> custom_drawing_call = std::nullopt;

			ImageDisplay();

		public:

			static std::unique_ptr<ImageDisplay> make();

			void update_image(image_provider provider);

			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;
			void compile() override;

			std::optional<skia_image_producer> get_image_wrapper();

			ImageDisplay& override_image_wrapper(std::optional<skia_image_producer> wrapper);

			ImageDisplay& update_custom_drawingcall(custom_step call);

			ImageDisplay& clear_custom_drawingcall();

			std::optional<custom_step> get_custom_drawingcall();

			ImageDisplay& update_batch(custom_step call, image_provider wrapper);

			ImageDisplay& set_image_size();
		};
	}
}

#endif