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
		using custom_step = std::function<void(SkCanvas*,double,double)>;

		class ImageDisplay : public  Drawable, utils::Lockable<ImageDisplay>, utils::Connectable<ImageDisplay> {
			int width = 100;
			int height = 100;
			using skia_image_producer = std::function<sk_sp<SkImage>(void)>;

			std::list<skia_image_producer> images_to_render;
			std::optional<custom_step> custom_drawing_call;
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
			inline void update_custom_drawingcall(custom_step call){
				std::lock_guard<std::mutex> g{ get_mutex() };
				custom_drawing_call = call;
			}
		};
	}
}

#endif