




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
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include <optional>
#include <nlohmann/json.hpp>
#include <iostream>
#include "utils/TheadPool.h"
#include "itkRegionOfInterestImageFilter.h"

#include "itkMultiResolutionImageRegistrationMethod.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkVersorRigid3DTransformOptimizer.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCheckerBoardImageFilter.h"
#include "itkCommand.h"
#include "itkShiftScaleImageFilter.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkImageDuplicator.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkCompositeTransform.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkRegionOfInterestImageFilter.h"





using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using MovingImageReaderType = itk::ImageFileReader<ImageType>;

int main(int argc, char **argv)
{

    
    if(argc!=4){
        if(argc>4 || argc == 1){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - input volume, (fixed)\n"
                      << "third parameter - output volume";
                      return 1;
            }
        if(argc == 2){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "third parameter - output volume";
                      return 1;
        }
        if(argc == 3){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "third parameter - output volume";
                      return 1;
        }
    }

    auto fixedImageReader = FixedImageReaderType::New();

    std::string dirName{argv[1]};
    //std::string dirName{"precious_phantom.mha"};
    fixedImageReader->SetFileName(dirName);


    try
    {
        fixedImageReader->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();

    //Custom 
     auto transform = TransformType::New();
     /*
    try
    {
    TransformType::VersorType versor;
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    auto rotation = Eigen::Vector3d::Random()*180.0;
    std::cout << "Random rotation: " << rotation << std::endl;
    matrix->SetRotation(rotation[0] * (3.14159265359 / 180), rotation[1] * (3.14159265359 / 180), rotation[2] * (3.14159265359 / 180));
    std::cout << "Matrix: " << matrix << std::endl;
    versor.Set(matrix->GetMatrix());

     std::cout << "transform set " << std::endl;
    auto origin = pointer2fixedimage->GetOrigin();
    auto offset = 
    auto center = origin;
    for(size_t i = 0; i< 3; ++i)
        center[i]+=offset[i];
     std::cout << "transform set " << std::endl;
    transform->SetCenter(center);
    transform->SetRotation(versor);

    std::cout << "transform set " << std::endl;
       }
  catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return 1;
    }*/

    using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(pointer2fixedimage);
    duplicator->Update();   
    ImageType::Pointer new_image = duplicator->GetOutput();

    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
   
    auto rotation = Eigen::Vector3d::Random()*180.0;
    matrix->SetRotation(rotation[0] * (3.14159265359 / 180), rotation[1] * (3.14159265359 / 180), rotation[2] * (3.14159265359 / 180));

    auto center_original_to_image_origin = pointer2fixedimage->GetDirection()*itk::Point<double,3>{{ (pointer2fixedimage->GetLargestPossibleRegion().GetSize()[0]/2.0)*pointer2fixedimage->GetSpacing()[0],
                                                                                                    (pointer2fixedimage->GetLargestPossibleRegion().GetSize()[1]/2.0)*pointer2fixedimage->GetSpacing()[1],
                                                                                                    (pointer2fixedimage->GetLargestPossibleRegion().GetSize()[2]/2.0)*pointer2fixedimage->GetSpacing()[2]}};


    std::cout << "center_original_to_image_origin :\n" << center_original_to_image_origin;                                                                                                
    auto center_original_to_transformed_image_origin = matrix->GetMatrix()*itk::Point<double,3>{{(pointer2fixedimage->GetLargestPossibleRegion().GetSize()[0]/2.0)*pointer2fixedimage->GetSpacing()[0],
                                                                                                    (pointer2fixedimage->GetLargestPossibleRegion().GetSize()[1]/2.0)*pointer2fixedimage->GetSpacing()[1],
                                                                                                    (pointer2fixedimage->GetLargestPossibleRegion().GetSize()[2]/2.0)*pointer2fixedimage->GetSpacing()[2]}};
    
    std::cout << "center_original_to_transformed_image_origin :\n" << center_original_to_transformed_image_origin;      
    auto transformed_image_origin = pointer2fixedimage->GetOrigin();
    for(size_t i = 0; i< 3; ++i)
        transformed_image_origin[i]+=center_original_to_image_origin[i]-center_original_to_transformed_image_origin[i];
    new_image->SetDirection(matrix->GetMatrix());
    new_image->SetOrigin(transformed_image_origin);


    std::cout << "old matrix: \n" << pointer2fixedimage->GetDirection() << "\nnew matrix:\n" << new_image->GetDirection();
    //matrix->SetRotation(info_registration.initial_rotation[0] * (3.14159265359 / 180), info_registration.initial_rotation[1] * (3.14159265359 / 180), info_registration.initial_rotation[2] * (3.14159265359 / 180));


  /*      using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageType,
                                          ImageType>;

    auto initialTransform = TransformType::New();
    TransformInitializerType::Pointer initializer =
        TransformInitializerType::New();
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(pointer2fixedimage);
    //initializer->SetMovingImage(info_registration.moving_image);

    try{
       initializer->InitializeTransform();
        std::cout << "here\n";
    }
    catch (...){
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    } 
    

       using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(pointer2fixedimage);
    duplicator->Update();

    ImageType::Pointer new_image = duplicator->GetOutput();

    new_image->SetOrigin(transform->GetOffset());
    new_image->SetDirection(transform->GetMatrix());

    using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    /*
    std::cout << "set set " << std::endl;
    auto resample = ResampleFilterType::New();
    resample->SetTransform(transform);
    resample->SetInput(pointer2fixedimage);
    resample->SetSize(pointer2fixedimage->GetLargestPossibleRegion().GetSize());
    resample->SetOutputOrigin(pointer2fixedimage->GetOrigin());
    resample->SetOutputSpacing(pointer2fixedimage->GetSpacing());
    resample->SetOutputDirection(pointer2fixedimage->GetDirection());
    resample->SetDefaultPixelValue(100);
    */

    std::cout << "set set set" << std::endl;
    using WriterType = itk::ImageFileWriter<ImageType>;
    std::cout << "here\n";
    auto writer = WriterType::New();
    writer->SetFileName(argv[3]);
    writer->SetInput(new_image);
    std::cout << "here\n";
    try{
        writer->Update();
        std::cout << "here\n";
    }
    catch (...){
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }    

    return 0;
}