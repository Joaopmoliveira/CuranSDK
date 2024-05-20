#ifndef CURAN_IMAGE_DISPLAY_HEADER_FILE_
#define CURAN_IMAGE_DISPLAY_HEADER_FILE_

#include <functional>
#include "definitions/UIdefinitions.h"
#include "Drawable.h"
#include "utils/Lockable.h"
#include <list>
#include "SignalProcessor.h"
#include "utils/MemoryUtils.h"
#include "ImageWrapper.h"

namespace curan {
namespace ui {

/*

*/

using custom_step = std::function<void(SkCanvas*, SkRect, SkRect)>;

class ImageDisplay : public  Drawable, public utilities::Lockable, public SignalProcessor<ImageDisplay>{
	int width = -1;
	int height = -1;
	bool compiled = false;
	std::optional<ImageWrapper> old_image = std::nullopt;
	std::optional<ImageWrapper>  images_to_render = std::nullopt;
	std::optional<custom_step> custom_drawing_call = std::nullopt;

	ImageDisplay();

public:

	static std::unique_ptr<ImageDisplay> make();

	void update_image(ImageWrapper provider);

	drawablefunction draw() override;
	callablefunction call() override;
	void compile() override;

	std::optional<ImageWrapper> get_image_wrapper();

	ImageDisplay& override_image_wrapper(ImageWrapper wrapper);

	ImageDisplay& update_custom_drawingcall(custom_step call);

	ImageDisplay& clear_custom_drawingcall();

	std::optional<custom_step> get_custom_drawingcall();

	ImageDisplay& update_batch(custom_step call, ImageWrapper wrapper);

	ImageDisplay& set_image_size();
};

}
}

#endif