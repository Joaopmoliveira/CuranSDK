#ifndef CURAN_IGTL_TO_ITK_CONVERTER_HEADER_FILE_
#define CURAN_IGTL_TO_ITK_CONVERTER_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "itkImage.h"
#include "igtlOSUtil.h"
#include "igtlImageMessage.h"

namespace curan{
    namespace image{

        void igtl2ITK_im_convert(const igtl::ImageMessage::Pointer& imageMessage, itk::Image<unsigned char, 3>::Pointer& image_to_render);

    }
}

#endif