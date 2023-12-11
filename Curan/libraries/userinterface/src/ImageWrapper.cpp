#include "userinterface/widgets/ImageWrapper.h"

namespace curan {
namespace ui {

ImageWrapper::ImageWrapper(std::shared_ptr<utilities::MemoryBuffer> in_buffer,size_t width,size_t height,SkColorType color, SkAlphaType opaqueness):
 buffer{in_buffer}{
		image_info = SkImageInfo::Make(width, height,color,opaqueness);
	    size_t row_size = width * sizeof(char);
	    auto wraped_skia_pixmap = SkPixmap{image_info,in_buffer->begin()->data(),row_size};
		image = SkSurfaces::WrapPixels(wraped_skia_pixmap)->makeImageSnapshot();
}

}
}