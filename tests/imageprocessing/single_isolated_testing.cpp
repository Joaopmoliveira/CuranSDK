#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkResampleImageFilter.h"
#include "itkEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"

constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;
using DicomPixelType = unsigned short;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension_in>;

using InterImageType = itk::Image<InterPixelType, Dimension_out>;

void exclude_row_matrix(Eigen::MatrixXd& matrix, size_t row_to_remove){
    int numRows = static_cast<int>(matrix.rows()) - 1;
    int numCols = static_cast<int>(matrix.cols());

    if(row_to_remove < numRows){
        matrix.block(row_to_remove,0,numRows-row_to_remove,numCols) = matrix.bottomRows(numRows-row_to_remove);
    }

    matrix.conservativeResize(numRows,numCols);
}


void calculate_image_centroid(const Eigen::Matrix<double,3,1>& volume_size_mm , Eigen::Matrix<double, 3, 1> &centroid, const Eigen::Matrix<double, 3, 3> &R_ImageToVolume, const Eigen::Matrix<double, 3, 1> &needle_tip_transformed_to_volume_space)
{
    Eigen::Matrix<double, 8, 3> vol_vertices_coords;

    vol_vertices_coords(0, 0) = 0.0;
    vol_vertices_coords(0, 1) = 0.0;
    vol_vertices_coords(0, 2) = 0.0;

    vol_vertices_coords(1, 0) = volume_size_mm[0];
    vol_vertices_coords(1, 1) = 0.0;
    vol_vertices_coords(1, 2) = 0.0;

    vol_vertices_coords(2, 0) = volume_size_mm[0];
    vol_vertices_coords(2, 1) = volume_size_mm[1];
    vol_vertices_coords(2, 2) = 0.0;

    vol_vertices_coords(3, 0) = 0.0;
    vol_vertices_coords(3, 1) = volume_size_mm[1];
    vol_vertices_coords(3, 2) = 0.0;

    vol_vertices_coords(4, 0) = 0.0;
    vol_vertices_coords(4, 1) = 0.0;
    vol_vertices_coords(4, 2) = volume_size_mm[2];

    vol_vertices_coords(5, 0) = volume_size_mm[0];
    vol_vertices_coords(5, 1) = 0.0;
    vol_vertices_coords(5, 2) = volume_size_mm[2];

    vol_vertices_coords(6, 0) = volume_size_mm[0];
    vol_vertices_coords(6, 1) = volume_size_mm[1];
    vol_vertices_coords(6, 2) = volume_size_mm[2];

    vol_vertices_coords(7, 0) = 0.0;
    vol_vertices_coords(7, 1) = volume_size_mm[1];
    vol_vertices_coords(7, 2) = volume_size_mm[2];

    double plane_equation[4];
    plane_equation[0] = R_ImageToVolume(0, 2);
    plane_equation[1] = R_ImageToVolume(1, 2);
    plane_equation[2] = R_ImageToVolume(2, 2);
    plane_equation[3] = -(R_ImageToVolume.col(2).transpose() * needle_tip_transformed_to_volume_space)(0, 0);

    constexpr double allowed_error = 1e-10;

    Eigen::MatrixXd intersections;
    intersections.resize(12, 3);

    size_t i = 0;
    for (size_t j = 0; j < vol_vertices_coords.rows(); ++j)
    {
        if (std::abs(vol_vertices_coords(j, 0)) < allowed_error){
            double majored_value = ((std::abs(plane_equation[0])>= allowed_error) ? plane_equation[0] : allowed_error);
            intersections(i, 0) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 1) = vol_vertices_coords(j, 1);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }
        if (std::abs(vol_vertices_coords(j, 1)) < allowed_error ){
            double majored_value = ((std::abs(plane_equation[1])>= allowed_error) ? plane_equation[1] : allowed_error);
            intersections(i, 1) = -(plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }

        if (std::abs(vol_vertices_coords(j, 2)) < allowed_error){
            double majored_value = ((std::abs(plane_equation[2])>= allowed_error) ? plane_equation[2] : allowed_error);
            intersections(i, 2) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 1) = vol_vertices_coords(j, 1);
            i = i + 1;
        }
    }

    size_t initial_number_of_points = static_cast<int>(intersections.rows())-1;
    bool no_rows = false;
    for (int i = initial_number_of_points; i >= 0 ; --i)
    {
        if (intersections(i, 0) > volume_size_mm[0] + allowed_error || intersections(i, 0) < 0.0 - allowed_error 
                 || intersections(i, 1) > volume_size_mm[1] + allowed_error || intersections(i, 1) < 0.0 - allowed_error 
                 || intersections(i, 2) > volume_size_mm[2] + allowed_error || intersections(i, 2) < 0.0 - allowed_error ){
            if(intersections.rows()==1){
                no_rows = true;
                break;
            }  
            exclude_row_matrix(intersections,i);
        }
    }
    
    centroid = (no_rows) ?  needle_tip_transformed_to_volume_space  : intersections.colwise().mean();
}

void obtain_rot_matrix_by_angles(Eigen::Matrix<double, 3, 3> &R_matrix, Eigen::Matrix<double, 3, 1> &rotaton_angles)
{

    typedef double T;

    auto a = rotaton_angles[0];
    auto b = rotaton_angles[1];
    auto c = rotaton_angles[2];

    R_matrix << cos(b) * cos(c), sin(a) * sin(b) * cos(c) - cos(a) * sin(c), cos(a) * sin(b) * cos(c) + sin(a) * sin(c),
        cos(b) * sin(c), sin(a) * sin(b) * sin(c) + cos(a) * cos(c), cos(a) * sin(b) * sin(c) - sin(a) * cos(c),
        -sin(b), sin(a) * cos(b), cos(a) * cos(b);
}

int main()
{
    try
    {

        Eigen::Matrix<double, 3, 1> volume_size{{1.0, 2.0, 3.0}};
        Eigen::Matrix<double, 3, 1> volume_spacing{{1.0, 1.0, 1.0}};
        Eigen::Matrix<double, 3, 1> image_orientation_angles;
        Eigen::Matrix<double, 3, 1> needle_tip{{0.5, 1.0, 1.5}};
        Eigen::Matrix<double, 3, 1> offset{{0.1, 0.1, 0.1}};

        Eigen::Matrix<double, 3, 3> R_ImageToVolume;
        double angle = 1.0;
        for (size_t ndx = 0; ndx < 4; ++ndx, angle += 1.0)
        {
            image_orientation_angles[0] = angle;
            image_orientation_angles[1] = angle;
            image_orientation_angles[2] = angle;

            needle_tip += offset;

            obtain_rot_matrix_by_angles(R_ImageToVolume, image_orientation_angles);
            std::cout << "Rotation matrix : \n"
                      << R_ImageToVolume << std::endl;
            Eigen::Matrix<double, 3, 1> centroid{{0.5, 1.0, 1.5}};
            calculate_image_centroid(volume_size, centroid, R_ImageToVolume, needle_tip);
            std::cout << "centroid : \n"
                      << centroid.transpose() << "\n";
        }
        return 0;
    }
    catch (...)
    {
        return 1;
    }
}