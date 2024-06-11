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
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include <optional>
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



ImageType::Pointer manipulate_input_image(ImageType::Pointer image)
{
    using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(image);
    duplicator->Update();

    ImageType::Pointer new_image = duplicator->GetOutput();
    auto origin = image->GetOrigin();

    auto new_origin = origin;
    new_origin[0] += 1111.0;
    new_origin[1] += 1111.0;
    new_origin[2] += 1111.0;

    new_image->SetOrigin(new_origin);

    auto direction = image->GetDirection();

    double euler_vector[3] = {1.1, -1.1, 1.1};

    double t1 = std::cos(euler_vector[2]);
    double t2 = std::sin(euler_vector[2]);
    double t3 = std::cos(euler_vector[1]);
    double t4 = std::sin(euler_vector[1]);
    double t5 = std::cos(euler_vector[0]);
    double t6 = std::sin(euler_vector[0]);

    direction(0, 0) = t1 * t3;
    direction(0, 1) = t1 * t4 * t6 - t2 * t5;
    direction(0, 2) = t2 * t6 + t1 * t4 * t5;

    direction(1, 0) = t2 * t3;
    direction(1, 1) = t1 * t5 + t2 * t4 * t6;
    direction(1, 2) = t2 * t4 * t5 - t1 * t6;

    direction(2, 0) = -t4;
    direction(2, 1) = t3 * t6;
    direction(2, 2) = t3 * t5;

    new_image->SetDirection(image->GetDirection()*direction);

    
    return new_image;
};

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

template<size_t sizes_for_piramids>
struct RegistrationParameters{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t,sizes_for_piramids> piramid_sizes;
    std::array<double,sizes_for_piramids> bluering_sizes;

    RegistrationParameters(size_t in_bin_numbers,
                           double in_relative_scales,
                           double in_learning_rate,
                           double in_sampling_percentage,
                           double in_relaxation_factor,
                           size_t in_convergence_window_size,
                           size_t in_optimization_iterations,
                           std::array<size_t,sizes_for_piramids> in_piramid_sizes,
                           std::array<double,sizes_for_piramids> in_bluering_sizes) : bin_numbers{in_bin_numbers},
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
    const Eigen::Vector3d &initial_rotation;
};

constexpr size_t size_info = 3;
using CompositeTransformType = itk::CompositeTransform<double, Dimension>;

std::tuple<double,CompositeTransformType::Pointer> solve_registration(const info_solve_registration &info_registration, RegistrationParameters<size_info>&& parameters)
{
    using InterpolatorType = itk::LinearInterpolateImageFunction<
            ImageRegistrationType,
            double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    //using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::MeanSquaresImageToImageMetricv4<ImageRegistrationType,ImageRegistrationType>;
    //using MetricType = itk::CorrelationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::JointHistogramMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    //using MetricType = itk::DemonsImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using MetricType = itk::ANTSNeighborhoodCorrelationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using RegistrationType = itk::ImageRegistrationMethodv4<ImageRegistrationType, ImageRegistrationType, TransformType>;
    auto outputCompositeTransform = CompositeTransformType::New();

    auto metric = MetricType::New();

    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    //metric->SetNumberOfHistogramBins(parameters.bin_numbers);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    itk::Size<3> size_of_itk_region;
    size_of_itk_region[0] = 4;
    size_of_itk_region[1] = 4;
    size_of_itk_region[2] = 4;
    metric->SetRadius(size_of_itk_region);
    if(info_registration.fixed_image_mask)
        metric->SetFixedImageMask(*info_registration.fixed_image_mask);
    if(info_registration.moving_image_mask)
        metric->SetMovingImageMask(*info_registration.moving_image_mask);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

     using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageRegistrationType,
                                          ImageRegistrationType>;

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    matrix->SetRotation(info_registration.initial_rotation[0] * (3.14159265359 / 180), info_registration.initial_rotation[1] * (3.14159265359 / 180), info_registration.initial_rotation[2] * (3.14159265359 / 180));
    TransformInitializerType::Pointer initializer =TransformInitializerType::New();
    initialTransform->SetMatrix(matrix->GetMatrix());
    initializer->SetTransform(initialTransform);

    initializer->SetFixedImage(info_registration.fixed_image);
    initializer->SetMovingImage(info_registration.moving_image);
    initializer->GeometryOn();
    initializer->InitializeTransform();

    outputCompositeTransform->AddTransform(initialTransform);

    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());

    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = parameters.relative_scales;
    optimizerScales[4] = parameters.relative_scales;
    optimizerScales[5] = parameters.relative_scales;

    optimizer->SetScales(optimizerScales);

    optimizer->SetNumberOfIterations(parameters.optimization_iterations);

    std::srand(std::time(nullptr)); 
    /*
        using GeneratorType = itk::Statistics::NormalVariateGenerator;
        auto generator = GeneratorType::New();
        generator->Initialize(std::rand());
        optimizer->SetNormalVariateGenerator(generator);
        optimizer->Initialize(10);
        optimizer->SetEpsilon(0.01);
        optimizer->SetMaximumIteration(4000);
    */

    
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

    try
    {
        registration->Update();
        //std::cout << "Registration completed!" << std::endl;
        //std::cout << "Optimizer stop condition: "
        //      << registration->GetOptimizer()->GetStopConditionDescription()
        //      << std::endl;
        outputCompositeTransform->AddTransform(registration->GetModifiableTransform());
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return {100.0, outputCompositeTransform};
    }
    return {optimizer->GetValue(), outputCompositeTransform};
}



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

    using FilterTypeThreshold = itk::ThresholdImageFilter<MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->ThresholdBelow(1);
    fixedSpatialObjectMask->SetImage(filter_threshold->GetOutput());
    fixedSpatialObjectMask->Update();
    pointer2fixedimage_registration = filter_threshold->GetOutput();
        using WriterType = itk::ImageFileWriter<MaskImageType>;
 
        auto writer = WriterType::New();
 
        writer->SetFileName("fixed_spacial_mask"+std::string(argv[3]));
        //writer->SetFileName("output_test_1.mha");

        writer->SetInput(filter_threshold->GetOutput());

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    

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

    using FilterTypeThreshold = itk::ThresholdImageFilter<MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->ThresholdBelow(1);
    movingSpatialObjectMask->SetImage(filter_threshold->GetOutput());
    movingSpatialObjectMask->Update();
    pointer2movingimage_registration = filter_threshold->GetOutput();

        using WriterType = itk::ImageFileWriter<MaskImageType>;
 
        auto writer = WriterType::New();
 
        writer->SetFileName("moving_spacial_mask"+std::string(argv[3]));
        //writer->SetFileName("output_test_1.mha");

        writer->SetInput(filter_threshold->GetOutput());

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
}


    std::ofstream myfile;
    myfile.open("results_of_fullscale_optimization.csv");
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    constexpr size_t local_permut = 1;
    std::array<size_t,local_permut> bin_numbers{20};
    std::array<double,local_permut> percentage_numbers{1};
    std::array<double,local_permut> relative_scales{1000.0};
    std::array<double,local_permut> learning_rate{7};
    std::array<double,local_permut> relaxation_factor{0.7};
    std::array<size_t,local_permut> optimization_iterations{2000};
    std::array<size_t,local_permut> convergence_window_size{20};
    std::array<std::array<size_t,size_info>,local_permut> piramid_sizes{{{3,2,1}}};
    std::array<std::array<double,size_info>,local_permut> bluering_sizes{{{2,1,0}}};

    constexpr size_t total_permutations = bin_numbers.size()*percentage_numbers.size()*relative_scales.size()*learning_rate.size()*relaxation_factor.size()*convergence_window_size.size()*piramid_sizes.size()*bluering_sizes.size();
    std::vector<std::tuple<double, CompositeTransformType::Pointer>> full_runs;

    std::array<Eigen::Vector3d,50> updated_initial_configs;
    
    std::printf("\nGenerating random initial guesses...\n");

    for(auto & vals : updated_initial_configs){
        vals = Eigen::Vector3d::Random()*180;
        std::printf("initial values: (%.2f %.2f %.2f)\n",vals[0],vals[1],vals[2]);
    }

{
auto run_parameterized_optimization = [&](size_t bins, size_t iters, double percentage, double relative_scales,double learning_rate, double relaxation_factor,size_t window_size, auto piramid_sizes, auto bluering_sizes){
        std::vector<std::tuple<double, CompositeTransformType::Pointer>> full_runs;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            for (const auto &initial_config : updated_initial_configs){
                curan::utilities::Job job{"solving registration",[&](){
                    auto solution = solve_registration(info_solve_registration{pointer2fixedimage_registration, pointer2movingimage_registration,fixedSpatialObjectMask,movingSpatialObjectMask,initial_config},RegistrationParameters{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs.emplace_back(solution);
                        std::cout << "cost: <<" << std::get<0>(solution) << "\n";
                    }              
                }};
                pool->submit(job);
            } 
        }
        return full_runs;
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
                                    for(const auto& blur_size : bluering_sizes) {
                                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                        full_runs = run_parameterized_optimization(bin_n,iters,percent_n,rel_scale,learn_rate,relax_factor,wind_size,pira_size,blur_size);
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
                                    }
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
    
    auto finalTransform = std::get<1>(full_runs[minimum_index])->GetFrontTransform();

    std::cout << "best estimated transform out of " << updated_initial_configs.size() << " is: \n" << std::get<1>(full_runs[minimum_index]) << std::endl ;


 
    {
        auto resample = itk::ResampleImageFilter<ImageType,ImageType>::New();
        resample->SetTransform(finalTransform);
        resample->SetInput(pointer2movingimage);
        resample->SetSize(pointer2fixedimage->GetLargestPossibleRegion().GetSize());
        resample->SetOutputOrigin(pointer2fixedimage->GetOrigin());
        resample->SetOutputSpacing(pointer2fixedimage->GetSpacing());
        resample->SetOutputDirection(pointer2fixedimage->GetDirection());
        resample->SetDefaultPixelValue(0);

 
        using WriterType = itk::ImageFileWriter<ImageType>;
 
        auto writer = WriterType::New();
 
        writer->SetFileName(argv[3]);
        //writer->SetFileName("output_test_1.mha");

        writer->SetInput(resample->GetOutput());

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
    }

    {
        auto writer = WriterType::New();
        using TransformHarderingType = itk::TransformGeometryImageFilter<ImageType,ImageType>;
        TransformHarderingType::Pointer harding = TransformHarderingType::New();
        harding->SetInput(pointer2movingimage);
        harding->SetTransform(finalTransform);

        writer->SetFileName("moving"+std::string{argv[3]});
        writer->SetInput(harding->GetOutput());

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
    }

    return 0;
}