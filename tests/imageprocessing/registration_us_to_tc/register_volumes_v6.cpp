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

constexpr size_t size_info = 1;

struct RegistrationParameters{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t,size_info> piramid_sizes;
    std::array<double,size_info> bluering_sizes;

    RegistrationParameters(size_t in_bin_numbers,
                           double in_relative_scales,
                           double in_learning_rate,
                           double in_sampling_percentage,
                           double in_relaxation_factor,
                           size_t in_convergence_window_size,
                           size_t in_optimization_iterations,
                           std::array<size_t,size_info> in_piramid_sizes,
                           std::array<double,size_info> in_bluering_sizes) : bin_numbers{in_bin_numbers},
                                                                                      relative_scales{in_relative_scales},
                                                                                      learning_rate{in_learning_rate},
                                                                                      sampling_percentage {in_sampling_percentage},
                                                                                      relaxation_factor{in_relaxation_factor},
                                                                                      convergence_window_size{in_convergence_window_size},
                                                                                      optimization_iterations{in_optimization_iterations},
                                                                                      piramid_sizes{in_piramid_sizes},
                                                                                      bluering_sizes{in_bluering_sizes}                                                    
    {

    }
};

struct info_solve_registration
{
    ImageRegistrationType::Pointer fixed_image;
    ImageRegistrationType::Pointer moving_image;
    std::optional<MaskType::Pointer> fixed_image_mask;
    std::optional<MaskType::Pointer> moving_image_mask;
    const Eigen::Matrix<double,4,4> &initial_rotation;
};



std::tuple<double,Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>> solve_registration(const info_solve_registration &info_registration, const RegistrationParameters& parameters)
{
    using InterpolatorType = itk::LinearInterpolateImageFunction<
            ImageRegistrationType,
            double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::MeanSquaresImageToImageMetricv4<ImageRegistrationType,ImageRegistrationType>;
    //using MetricType = itk::CorrelationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::JointHistogramMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::DemonsImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::ANTSNeighborhoodCorrelationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using RegistrationType = itk::ImageRegistrationMethodv4<ImageRegistrationType, ImageRegistrationType, TransformType>;

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    metric->SetNumberOfHistogramBins(parameters.bin_numbers);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);

    if(info_registration.fixed_image_mask)
        metric->SetFixedImageMask(*info_registration.fixed_image_mask);
    if(info_registration.moving_image_mask)
        metric->SetMovingImageMask(*info_registration.moving_image_mask);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    itk::Vector<double,3> origin;
    itk::Matrix<double> direction;
    for(size_t row = 0; row < 3; ++row){
        origin[row] = info_registration.initial_rotation(row,3);
        for(size_t col = 0; col < 3; ++col){
            direction(row,col) = info_registration.initial_rotation(row,col);
        }
    }

    //sanity check 
    if(!(info_registration.initial_rotation.block<3,3>(0,0).transpose()*info_registration.initial_rotation.block<3,3>(0,0)).isDiagonal()){
        std::cout << "failure to initialize rotation matrix...\n";
        std::cout << "values are: \n";
        std::cout << info_registration.initial_rotation.block<3,3>(0,0) << std::endl;
        std::cout << "the multiplication with itself is:";
        std::cout << info_registration.initial_rotation.block<3,3>(0,0).transpose()*info_registration.initial_rotation.block<3,3>(0,0) << std::endl;
        throw std::runtime_error("failure to initialize the rotation matrix");
    }
    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);
    
    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(initialTransform->GetNumberOfParameters());

    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = parameters.relative_scales;
    optimizerScales[4] = parameters.relative_scales;
    optimizerScales[5] = parameters.relative_scales;

    optimizer->SetScales(optimizerScales);

    optimizer->SetNumberOfIterations(parameters.optimization_iterations);

    std::srand(std::time(nullptr)); 
    
    optimizer->SetLearningRate(parameters.learning_rate);
    optimizer->SetMinimumStepLength(0.0001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{parameters.convergence_window_size};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(parameters.relaxation_factor);

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(size_info);
    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(size_info);

    for(size_t i = 0; i < size_info; ++i){
        shrinkFactorsPerLevel[i] = parameters.piramid_sizes[i];
        smoothingSigmasPerLevel[i] = parameters.bluering_sizes[i];
        //std::printf("shrinkFactorsPerLevel(%llu) smoothingSigmasPerLevel(%.4f)\n",shrinkFactorsPerLevel[i],smoothingSigmasPerLevel[i]);
    }

    registration->SetNumberOfLevels(size_info);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::REGULAR;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(parameters.sampling_percentage);

    registration->MetricSamplingReinitializeSeed(std::rand());
    registration->SetInPlace(false);

    try{
        registration->Update();
    } catch (const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return {100.0, Eigen::Matrix<double,4,4>::Identity(),info_registration.initial_rotation};
    }
    TransformType::Pointer final_registration = registration->GetModifiableTransform();
    Eigen::Matrix<double,4,4> final_transformation = Eigen::Matrix<double,4,4>::Identity();
    for(size_t row = 0; row < 3; ++row){
        final_transformation(row,3) = final_registration->GetOffset()[row];
        for(size_t col = 0; col < 3; ++col){
            final_transformation(row,col) = final_registration->GetMatrix()(row,col);
        }
    }

    //std::cout << "final_transform vs initial transform :\n====================\n" ;
    //std::cout << "final: \n" << final_registration << std::endl;
    //std::cout << "initial transform: \n" << initialTransform << std::endl;
    //std::cout << "\n====================\n" ;

    return {optimizer->GetValue(), final_transformation,info_registration.initial_rotation};
}

void print_image_info(itk::Image<PixelType,3>::Pointer image, std::string name){
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

int modify_image_with_transform(Eigen::Matrix<double,4,4> transform,itk::Image<unsigned char,Dimension>::Pointer image){
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
    return 1;
};

int modify_image_with_transform(Eigen::Matrix<double,4,4> transform,ImageType::Pointer image){
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
    return 1;
};

int print_image_with_transform(ImageType::Pointer image,std::string image_path){
    using WriterType = itk::ImageFileWriter<ImageType>;
    auto writer = WriterType::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    writer->Update();
    return 1;
};

int main(int argc, char **argv)
{

    /*
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
    */ 
    try{
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();

    //std::string dirName{argv[1]};
    std::string dirName{"precious_with_mask_manual.mha"};
    fixedImageReader->SetFileName(dirName);

    //std::string dirName2{argv[2]};
    std::string dirName2{"usreconstruction_with_mask.mha"};
    movingImageReader->SetFileName(dirName2);

    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
    }
    catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
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
    try{
        meshSource->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }
    
    pointer2fixedimage_registration = castfilter->GetOutput();
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
    try{
        meshSource->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }
    pointer2movingimage_registration = castfilter->GetOutput();
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

    auto transform = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha) ,-std::sin(alpha) , 0.0 ,
                                        std::sin(alpha) , std::cos(alpha) , 0.0 ,
                                            0.0         ,      0.0        , 1.0 ;
        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction;
        return T_rotation_extra;
    };

    auto transform_flipped_principal_component = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha) ,-std::sin(alpha) , 0.0 ,
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

    std::ofstream myfile;
    myfile.open("results_of_fullscale_optimization.csv");
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    constexpr size_t local_permut = 1;
    std::array<size_t,local_permut> bin_numbers{50};
    std::array<double,local_permut> percentage_numbers{1};
    std::array<double,local_permut> relative_scales{1000.0};
    std::array<double,local_permut> learning_rate{7};
    std::array<double,local_permut> relaxation_factor{0.7};
    std::array<size_t,local_permut> optimization_iterations{2000};
    std::array<size_t,local_permut> convergence_window_size{20};
    std::array<std::array<size_t,size_info>,local_permut> piramid_sizes{{{1}}};
    std::array<std::array<double,size_info>,local_permut> bluering_sizes{{{0}}};

    constexpr size_t total_permutations = bin_numbers.size()*percentage_numbers.size()*relative_scales.size()*learning_rate.size()*relaxation_factor.size()*convergence_window_size.size()*piramid_sizes.size()*bluering_sizes.size();
    std::vector<std::tuple<double,Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>>> full_runs;
    
    std::printf("\nGenerating initial guesses...\n");
    std::vector<Eigen::Matrix<double,4,4>> angles_regular;
    for(double angle = 0; angle < 360.0 ; angle+=10.0){
        angles_regular.push_back(transform(angle));
        angles_regular.push_back(transform_flipped_principal_component(angle));
    }

    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage_registration);
    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage_registration);
    
    auto run_parameterized_optimization = [&](size_t bins, size_t iters, double percentage, double relative_scales,double learning_rate, double relaxation_factor,size_t window_size, auto piramid_sizes, auto bluering_sizes){
        std::vector<std::tuple<double, Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>>> full_runs_inner;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            for (const auto &initial_config : angles_regular){
                curan::utilities::Job job{"solving registration",[&](){
                    auto solution = solve_registration(info_solve_registration{pointer2fixedimage_registration, pointer2movingimage_registration,std::nullopt,std::nullopt,initial_config},RegistrationParameters{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs_inner.emplace_back(solution);
                    }              
                }};
                pool->submit(job);
            } 
        }
        return full_runs_inner;
    };

    size_t total_runs = 0;
    for(const auto& bin_n : bin_numbers)
        for(const auto& percent_n : percentage_numbers)
            for(const auto& rel_scale : relative_scales)
                for(const auto& learn_rate : learning_rate)
                    for(const auto& relax_factor : relaxation_factor)
                        for(const auto& wind_size : convergence_window_size)
                            for(const auto& pira_size : piramid_sizes)
                                for(const auto& iters : optimization_iterations)
                                    for(const auto& blur_size : bluering_sizes){
                                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                        auto paralel_solutions = run_parameterized_optimization(bin_n,iters,percent_n,rel_scale,learn_rate,relax_factor,wind_size,pira_size,blur_size);
                                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                        for(auto&& run : full_runs){
                                            myfile << total_runs << "," << bin_n << "," << percent_n << "," << rel_scale << "," << learn_rate << "," << relax_factor << "," << wind_size << ", {";
                                            for(const auto& val : pira_size)
                                                myfile << val << ";";
                                            myfile << "}, {";
                                            for(const auto& val : blur_size)
                                                myfile << val << ";"; 
                                            myfile <<"}," << std::get<0>(run) << "," << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
                                        }
                                        ++total_runs;
                                        full_runs.insert(std::end(full_runs),std::begin(paralel_solutions),std::end(paralel_solutions));          
                                    } 

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = std::numeric_limits<double>::max();
    for (const auto &possible_best_solution : full_runs){
        if (minimum_val > std::get<0>(possible_best_solution)){
            minimum_index = current_index;
            minimum_val = std::get<0>(possible_best_solution);
        }
        ++current_index;
    }


    std::printf("Choosen cost: %.2f\n",minimum_val);
    
    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    std::cout << "best estimated transform out of " << full_runs.size() << " is: \n" << std::get<1>(full_runs[minimum_index]) << std::endl ;

    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage);
    print_image_with_transform(pointer2fixedimage,"fixed_image_moved_to_origin.mha");

    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_image_moved_to_origin.mha");

    modify_image_with_transform(finalTransform.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct.mha");

    modify_image_with_transform(std::get<2>(full_runs[minimum_index]).inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_initial_guess.mha");

    return 0;
}
    catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return 1;
    } catch(...){
        std::cout << "Some exception was thrown.\n";
        return 1;
    }



/*
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
*/
    return 0;
}