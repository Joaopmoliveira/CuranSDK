#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkCorrelationImageToImageMetricv4.h"
#include "itkJointHistogramMutualInformationImageToImageMetricv4.h"
#include "itkDemonsImageToImageMetricv4.h"
#include <itkANTSNeighborhoodCorrelationImageToImageMetricv4.h>
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMeshFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include <optional>
#include "itkBinaryMask3DMeshSource.h"
#include <type_traits>
#include <nlohmann/json.hpp>
#include <iostream>
#include "utils/TheadPool.h"
#include "itkRegionOfInterestImageFilter.h"
#include <itkTransformGeometryImageFilter.h>
#include <fstream>
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkNormalVariateGenerator.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkImageMaskSpatialObject.h"
#include "itkThresholdImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"


const double pi = std::atan(1) * 4;

using PixelType = float;
using RegistrationPixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;

using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<PixelType, Dimension>;
using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
using WriterType = itk::ImageFileWriter<OutputImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using MaskType = itk::ImageMaskSpatialObject<Dimension>;
using MovingImageReaderType = itk::ImageFileReader<ImageType>;

auto print_image_info = [](itk::Image<PixelType,3>::Pointer image, std::string name){
        std::cout << "-------------------\n";
        std::cout << "(" << name << ") :\n -------------------\n";
        std::cout << "direction:\n";
        auto direction = image->GetDirection();
        for(size_t i = 0; i < 3; ++i){
            for(size_t j = 0; j < 3; ++j)
                std::cout <<  direction(i,j) << " ";
            std::cout << "\n";
        }
        auto origin = image->GetOrigin();
        std::printf("\norigin: (%f %f %f)",image->GetOrigin()[0],image->GetOrigin()[1],image->GetOrigin()[2]);
        std::printf("\nspacing: (%f %f %f)",image->GetSpacing()[0],image->GetSpacing()[1],image->GetSpacing()[2]);
        std::printf("\nsize: (%d %d %d)",(int)image->GetLargestPossibleRegion().GetSize()[0],(int)image->GetLargestPossibleRegion().GetSize()[1],(int)image->GetLargestPossibleRegion().GetSize()[2]);
        std::cout << "\n-------------------\n";
};

int main(int argc, char **argv)
{

    
    if(argc!=4){
        if(argc>4 || argc == 1){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - input volume, (fixed)\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
            }
        if(argc == 2){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
        }
        if(argc == 3){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - "<< std::string(argv[2]) << "\n"
                      << "third parameter - output volume";
                      return 1;
        }
    }
    
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();

    std::string dirName{argv[1]};
    fixedImageReader->SetFileName(dirName);

    std::string dirName2{argv[2]};
    movingImageReader->SetFileName(dirName2);

    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
    auto fixedSpatialObjectMask = MaskType::New();
    ImageRegistrationType::Pointer pointer2fixedimage_registration;
    ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();
    auto movingSpatialObjectMask = MaskType::New();
    ImageRegistrationType::Pointer pointer2movingimage_registration;

    Eigen::Matrix<double,4,4> T_origin_fixed = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,4,4> T_origin_moving = Eigen::Matrix<double,4,4>::Identity();

{
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(pointer2fixedimage);
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<MaskPixelType>::max());

    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    auto castfilter = CastFilterType::New();
    castfilter->SetInput(rescale->GetOutput());

    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->SetInsideValue(1);   
    filter_threshold->SetLowerThreshold(1);
    filter_threshold->SetUpperThreshold(255);
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(1);
    meshSource->SetInput(filter_threshold->GetOutput());
    meshSource->Update();
    auto mesh = meshSource->GetOutput();
    Eigen::Matrix<double,Eigen::Dynamic,3> points_in_matrix_form = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    PointsIterator end = mesh->GetPoints()->End();
    size_t index = 0;
    while (pointIterator != end)
    {
        auto p = pointIterator->Value();
        points_in_matrix_form(index,0) = p[0];
        points_in_matrix_form(index,1) = p[1];
        points_in_matrix_form(index,2) = p[2];
        ++pointIterator;
        ++index;
    }
    Eigen::Matrix<double,3,1> center_of_fixed_image = points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double,1,3> to_subtract = center_of_fixed_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;    
    Eigen::Matrix<double,3,3> covariance_of_fixed_surface = points_in_matrix_form.transpose()*points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd( covariance_of_fixed_surface, Eigen::ComputeFullV | Eigen::ComputeFullU );
    assert(svd.computeU());
    auto rotation_of_fixed_image = svd.matrixU();
    rotation_of_fixed_image.col(2) = rotation_of_fixed_image.col(0).cross(rotation_of_fixed_image.col(1));
    T_origin_fixed.block<3,3>(0,0) = rotation_of_fixed_image;
    T_origin_fixed.block(0,3,3,1) = center_of_fixed_image;
}

{
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(pointer2movingimage);
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<MaskPixelType>::max());

    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    auto castfilter = CastFilterType::New();
    castfilter->SetInput(rescale->GetOutput());

    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->SetInsideValue(1);   
    filter_threshold->SetLowerThreshold(1);
    filter_threshold->SetUpperThreshold(255);
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(1);
    meshSource->SetInput(filter_threshold->GetOutput());
    meshSource->Update();
    auto mesh = meshSource->GetOutput();
    Eigen::Matrix<double,Eigen::Dynamic,3> points_in_matrix_form = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    PointsIterator end = mesh->GetPoints()->End();
    size_t index = 0;
    while (pointIterator != end)
    {
        auto p = pointIterator->Value();
        points_in_matrix_form(index,0) = p[0];
        points_in_matrix_form(index,1) = p[1];
        points_in_matrix_form(index,2) = p[2];
        ++pointIterator;
        ++index;
    }
    Eigen::Matrix<double,3,1> center_of_moving_image = points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double,1,3> to_subtract = center_of_moving_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;    
    Eigen::Matrix<double,3,3> covariance_of_moving_surface = points_in_matrix_form.transpose()*points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd( covariance_of_moving_surface, Eigen::ComputeFullV | Eigen::ComputeFullU );
    assert(svd.computeU());
    auto rotation_of_moving_image = svd.matrixU();
    rotation_of_moving_image.col(2) = rotation_of_moving_image.col(0).cross(rotation_of_moving_image.col(1));
    T_origin_moving.block<3,3>(0,0) = rotation_of_moving_image;
    T_origin_moving.block(0,3,3,1) = center_of_moving_image;
}

    // The last axis which is poorly defined is the axis the Z axis, e.g. the first well defined orientation is X, 
    // along the principal axis, followed by the Y axis followed by the Z axis, thus this is the orientation we use 
    // to generate our initial transforms

    std::cout << "T_origin_fixed: \n" << T_origin_fixed << std::endl;
    std::cout << "T_origin_moving: \n" << T_origin_moving << std::endl;

    Eigen::Matrix<double,4,4> Timage_origin_fixed = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,4,4> Timage_origin_moving = Eigen::Matrix<double,4,4>::Identity();

    for(size_t row = 0; row < 3; ++row){
        Timage_origin_fixed(row,3) = pointer2fixedimage->GetOrigin()[row];
        Timage_origin_moving(row,3) = pointer2movingimage->GetOrigin()[row];
        for(size_t col = 0; col < 3; ++col){
            Timage_origin_fixed(row,col) = pointer2fixedimage->GetDirection()(row,col);
            Timage_origin_moving(row,col) = pointer2movingimage->GetDirection()(row,col);
        }
    }

    std::cout << "Timage_origin_fixed: \n" << Timage_origin_fixed << std::endl;
    std::cout << "Timage_origin_moving: \n" << Timage_origin_moving << std::endl;



    auto print_image_with_transform = [](Eigen::Matrix<double,4,4> transform,ImageType::Pointer image,std::string path_to_output){
        itk::Point<double,3> origin;
        itk::Matrix<double> direction;
        for(size_t row = 0; row < 3; ++row){
            origin[row] = transform(row,3);
            for(size_t col = 0; col < 3; ++col){
                direction(row,col) = transform(row,col);
            }
        }
        image->SetOrigin(origin);
        image->SetDirection(direction);

        auto writer = WriterType::New();
        writer->SetFileName(path_to_output);
        writer->SetInput(image);
        try
        {
            writer->Update();
        }
        catch (...)
        {
            std::cout << "Failed to write image";
            return 1;
        }
        return 0;
    };

    auto transform = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha) , std::sin(alpha) , 0.0 ,
                                        std::sin(alpha) , std::cos(alpha) , 0.0 ,
                                            0.0         ,      0.0        , 1.0 ;
        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction;
        return T_rotation_extra;
    };

    auto transform_flipped_principal_component = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha) , std::sin(alpha) , 0.0 ,
                                        std::sin(alpha) , std::cos(alpha) , 0.0 ,
                                            0.0         ,      0.0        , 1.0 ;

        Eigen::Matrix<double,3,3> flipping_direction;
        flipping_direction << 1.0         ,      0.0        , 0.0 ,
                              0.0         ,     -1.0        , 0.0 ,
                              0.0         ,      0.0        ,-1.0 ;

        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction*flipping_direction;
        return T_rotation_extra;
    };

    if(print_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_1_"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage,"fixed_1_"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform(0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_1_alpha_0"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform(0.0174532925*90.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_1_alpha_90"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform(0.0174532925*180.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_1_alpha_180"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform(0.0174532925*270.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_1_alpha_270"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform_flipped_principal_component(0.0174532925*0.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_flipped_alpha_0"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform_flipped_principal_component(0.0174532925*90.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_flipped_alpha_90"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform_flipped_principal_component(0.0174532925*180.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_flipped_alpha_180"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    if(print_image_with_transform(transform_flipped_principal_component(0.0174532925*270.0)*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage,"moving_flipped_alpha_270"+std::string{argv[3]})){
        std::cout << "Failure to print image" << std::endl;
        return 1;
    }

    return 0;
}