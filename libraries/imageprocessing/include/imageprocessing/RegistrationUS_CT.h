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

    enum CentroidComputation{
        FROM_POINT_CLOUD,
        CENTER_OF_3D_IMAGE
    };

    size_t number_of_roi_regions;
    MeshSelection fixed_image_selection_policy;
    MeshSelection moving_image_selection_policy;
    CentroidComputation centroid_computation;

    RegistrationConfiguration(size_t in_number_of_roi_regions,
                                MeshSelection in_fixed_image_selection_policy,
                                MeshSelection in_moving_image_selection_policy,
                                CentroidComputation in_centroid_computation_strategy) : number_of_roi_regions{in_number_of_roi_regions} , 
                                                                                fixed_image_selection_policy{in_fixed_image_selection_policy}, 
                                                                                moving_image_selection_policy{in_moving_image_selection_policy},
                                                                                centroid_computation{in_centroid_computation_strategy} 
    {};
};

int register_volumes(ImageType::Pointer pointer2inputfixedimage, ImageType::Pointer pointer2inputmovingimage,const RegistrationConfiguration& configuration);

#endif