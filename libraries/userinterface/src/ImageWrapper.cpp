#include "userinterface/widgets/ImageWrapper.h"

namespace curan {
namespace ui {

ImageWrapper::ImageWrapper(std::shared_ptr<utilities::MemoryBuffer> in_buffer,size_t width,size_t height,SkColorType color, SkAlphaType opaqueness):
 buffer{in_buffer}{
		image_info = SkImageInfo::Make(width, height,color,opaqueness);
		size_t pixel_size = (size_t) std::ceil(in_buffer->begin()->size()/(double) (width*height)) ;
	    size_t row_size = width * pixel_size;
	    auto wraped_skia_pixmap = SkPixmap{image_info,in_buffer->begin()->data(),row_size};
		image = SkSurfaces::WrapPixels(wraped_skia_pixmap)->makeImageSnapshot();
}

}
}