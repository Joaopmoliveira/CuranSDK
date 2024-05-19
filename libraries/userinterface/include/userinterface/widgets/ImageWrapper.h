#ifndef CURAN_IMAGE_WRAPPER_HEADER_FILE_
#define CURAN_IMAGE_WRAPPER_HEADER_FILE_

#include "definitions/UIdefinitions.h"
#include "utils/MemoryUtils.h"

namespace curan {
namespace ui {

/*

*/

struct ImageWrapper{
	std::shared_ptr<utilities::MemoryBuffer> buffer;
	SkImageInfo image_info;
	sk_sp<SkImage> image;
	ImageWrapper(std::shared_ptr<utilities::MemoryBuffer> buffer,size_t width,size_t height,SkColorType color = SkColorType::kGray_8_SkColorType, SkAlphaType opaqueness = SkAlphaType::kOpaque_SkAlphaType);
};

}
}

#endif