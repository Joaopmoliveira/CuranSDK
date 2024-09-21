#ifndef CURAN_REGISTRATION_HEADER_FILE_
#define CURAN_REGISTRATION_HEADER_FILE_

#include <string_view>
#include "itkImage.h"

using PixelType = float;
using RegistrationPixelType = PixelType;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

struct RegistrationConfiguration{

    enum MeshSelection{
        SELECT_VERTICES_POINTING_OUTWARDS,
        SELECT_VERTICES_POINTING_INWARDS
    };

    size_t number_of_roi_regions;
    MeshSelection fixed_image_selection_policy;
    MeshSelection moving_image_selection_policy;

    RegistrationConfiguration(size_t in_number_of_roi_regions,
                                MeshSelection in_fixed_image_selection_policy,
                                MeshSelection in_moving_image_selection_policy) : number_of_roi_regions{in_number_of_roi_regions} , 
                                                                                fixed_image_selection_policy{in_fixed_image_selection_policy}, 
                                                                                moving_image_selection_policy{in_moving_image_selection_policy}
    {};
};

int register_volumes(ImageType::Pointer pointer2inputfixedimage, ImageType::Pointer pointer2inputmovingimage,const RegistrationConfiguration& configuration);

#endif