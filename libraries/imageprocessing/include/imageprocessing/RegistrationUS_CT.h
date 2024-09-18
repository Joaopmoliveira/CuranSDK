#ifndef CURAN_REGISTRATION_HEADER_FILE_
#define CURAN_REGISTRATION_HEADER_FILE_

#include <string_view>
#include "itkImage.h"

using PixelType = float;
using RegistrationPixelType = PixelType;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

int register_volumes(ImageType::Pointer pointer2inputfixedimage, ImageType::Pointer pointer2inputmovingimage);

#endif