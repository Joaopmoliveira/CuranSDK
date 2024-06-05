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
#include <fstream>

const double pi = std::atan(1) * 4;

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::Euler3DTransform<double>;
using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;
using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<PixelType, Dimension>;
using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
using WriterType = itk::ImageFileWriter<OutputImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using MovingImageReaderType = itk::ImageFileReader<ImageType>;
using InterpolatorType = itk::LinearInterpolateImageFunction<
            ImageType,
            double>;


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
}

constexpr size_t number_of_piramids = 4;

struct RegistrationParameters{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t,number_of_piramids> piramid_sizes;
    std::array<size_t,number_of_piramids> bluering_sizes;
};

struct info_solve_registration
{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    const Eigen::Vector3d &initial_rotation;
};

std::tuple<double,TransformType::Pointer,TransformType::Pointer> solve_registration(const info_solve_registration &info_registration, const RegistrationParameters& parameters)
{
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    metric->SetNumberOfHistogramBins(parameters.bin_numbers);

    metric->SetUseMovingImageGradientFilter(true);
    metric->SetUseFixedImageGradientFilter(true);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

     using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageType,
                                          ImageType>;

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    matrix->SetRotation(info_registration.initial_rotation[0] * (3.14159265359 / 180), info_registration.initial_rotation[1] * (3.14159265359 / 180), info_registration.initial_rotation[2] * (3.14159265359 / 180));
    TransformInitializerType::Pointer initializer =TransformInitializerType::New();
    initialTransform->SetMatrix(matrix->GetMatrix());
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(info_registration.fixed_image);
    initializer->SetMovingImage(info_registration.moving_image);
    initializer->InitializeTransform();
    initializer->GeometryOn();

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
    optimizer->SetLearningRate(parameters.learning_rate);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{parameters.convergence_window_size};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(parameters.relaxation_factor);

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(number_of_piramids);
    shrinkFactorsPerLevel[0] = parameters.piramid_sizes[0];
    shrinkFactorsPerLevel[1] = parameters.piramid_sizes[1];
    shrinkFactorsPerLevel[2] = parameters.piramid_sizes[2];
    shrinkFactorsPerLevel[3] = parameters.piramid_sizes[2];

    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(number_of_piramids);
    smoothingSigmasPerLevel[0] = parameters.bluering_sizes[0];
    smoothingSigmasPerLevel[1] = parameters.bluering_sizes[1];
    smoothingSigmasPerLevel[2] = parameters.bluering_sizes[2];
    smoothingSigmasPerLevel[3] = parameters.bluering_sizes[3];

    registration->SetNumberOfLevels(number_of_piramids);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::RANDOM;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(parameters.sampling_percentage);

    std::srand(std::time(nullptr)); 
    registration->MetricSamplingReinitializeSeed(std::rand());

    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        TransformType::Pointer finalTransform = registration->GetModifiableTransform();
        return {100.0, finalTransform,initialTransform};
    }

    TransformType::Pointer finalTransform = registration->GetModifiableTransform();
    return {optimizer->GetCurrentMetricValue(), finalTransform,initialTransform};
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
    ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

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

    print_image_info(pointer2fixedimage,"fixed image");
    print_image_info(pointer2movingimage,"moving image");

    std::array<Eigen::Vector3d,27> initial_configs;
    
    std::printf("\nGenerating random initial guesses...\n");
    double advancement_x = 6.28318530718/(std::cbrt(initial_configs.size())-1);
    double advancement_y = 6.28318530718/(std::cbrt(initial_configs.size())-1);
    double advancement_z = 6.28318530718/(std::cbrt(initial_configs.size())-1);
    double current_x = 0;
    double current_y = 0;
    double current_z = 0;

    for(auto & vals : initial_configs){
        vals[0] = current_x;
        vals[1] = current_y;
        vals[2] = current_z;
        current_z += advancement_z;
        if(current_z>6.28318530718){
            current_z = 0.0;
            current_y +=advancement_y;
        }

        if(current_y>6.28318530718){
            current_z = 0.0;
            current_y = 0.0;
            current_x +=advancement_x;
        } 

        std::printf("initial values: (%.2f %.2f %.2f)\n",vals[0],vals[1],vals[2]);
    }

    std::ofstream myfile;
    myfile.open("results_of_fullscale_optimization.csv");
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    constexpr size_t local_permut = 1;
/*
    std::array<size_t,local_permut> bin_numbers{10,20,50};
    std::array<double,local_permut> percentage_numbers{0.05,0.1,0.4};
    std::array<double,local_permut> relative_scales{2000.0,1000.0,800.0};
    std::array<double,local_permut> learning_rate{10,8,5};
    std::array<double,local_permut> relaxation_factor{0.9,0.7,0.5};
    std::array<size_t,local_permut> optimization_iterations{100,300,500};
    std::array<size_t,local_permut> convergence_window_size{5,10,20};
    std::array<std::array<size_t,4>,local_permut> piramid_sizes{{{4,3,1,0},{8,6,2,0},{8,4,2,0}}};
    std::array<std::array<size_t,4>,local_permut> bluering_sizes{{{6,3,2,0},{6,3,1,0},{8,4,2,0}}};
*/
    std::array<size_t,local_permut> bin_numbers{50};
    std::array<double,local_permut> percentage_numbers{0.1};
    std::array<double,local_permut> relative_scales{1000.0};
    std::array<double,local_permut> learning_rate{8};
    std::array<double,local_permut> relaxation_factor{0.7};
    std::array<size_t,local_permut> optimization_iterations{500};
    std::array<size_t,local_permut> convergence_window_size{20};
    std::array<std::array<size_t,4>,local_permut> piramid_sizes{{{8,4,2,0}}};
    std::array<std::array<size_t,4>,local_permut> bluering_sizes{{{8,4,2,0}}};

    constexpr size_t total_permutations = bin_numbers.size()*percentage_numbers.size()*relative_scales.size()*learning_rate.size()*relaxation_factor.size()*convergence_window_size.size()*piramid_sizes.size()*bluering_sizes.size();

    auto run_parameterized_optimization = [&](size_t bins, size_t iters, double percentage, double relative_scales,double learning_rate, double relaxation_factor,size_t window_size, std::array<size_t,4> piramid_sizes, std::array<size_t,4> bluering_sizes){
        std::vector<std::tuple<double, TransformType::Pointer,TransformType::Pointer>> full_runs;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(4,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            for (const auto &initial_config : initial_configs){
                curan::utilities::Job job{"solving registration",[&](){
                    auto solution = solve_registration({pointer2fixedimage, pointer2movingimage, initial_config},{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs.emplace_back(solution);
                        std::cout << ".";
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
                                        auto cost = run_parameterized_optimization(bin_n,iters,percent_n,rel_scale,learn_rate,relax_factor,wind_size,pira_size,blur_size);
                                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                        for(auto&& run : cost){
                                            myfile << total_runs << "," << bin_n << "," << percent_n << "," << rel_scale << "," << learn_rate << "," << relax_factor << "," << wind_size << ", {";
                                            for(const auto& val : pira_size)
                                                myfile << val << ";";
                                            myfile << "}, {";
                                            for(const auto& val : blur_size)
                                                myfile << val << ";"; 
                                            myfile <<"}," << std::get<0>(run) << "," << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
                                            std::printf("\n==(%d/%d)==\n",(int)total_runs,(int)total_permutations);
                                        }
                                        ++total_runs;
                                    }
/*
    std::printf("Choosen cost: %.2f\n",minimum_val);
    
    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    std::cout << "best estimated transform out of " << initial_configs.size() << " is: \n" << std::get<1>(full_runs[minimum_index]) << " and other is: " << std::get<2>(full_runs[minimum_index]) ;

    using CompositeTransformType = itk::CompositeTransform<double, Dimension>;
    CompositeTransformType::Pointer outputCompositeTransform =
    CompositeTransformType::New();
    //outputCompositeTransform->AddTransform(std::get<2>(full_runs[minimum_index]));
    //outputCompositeTransform->AddTransform());

    using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
 
    {
        auto resample = ResampleFilterType::New();
        resample->SetTransform(std::get<1>(full_runs[minimum_index]));
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
 
        writer->SetFileName("moving"+std::string{argv[3]});
        writer->SetInput(pointer2movingimage);

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
    }
*/
    return 0;
}