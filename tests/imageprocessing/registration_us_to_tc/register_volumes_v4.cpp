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

class CommandType : public itk::Command
{
public:
    using Self = CommandType;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandType() = default;

public:
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using OptimizerPointer = const OptimizerType *;
    Eigen::Matrix<double, 4, 4> moving_homogenenous;
    TransformType::Pointer initialTransform;

    void Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }
    void Execute(const itk::Object *object, const itk::EventObject &event) override
    {
    }
};

struct info_solve_registration
{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    const Eigen::Vector3d &initial_rotation;
};

std::tuple<double, TransformType::Pointer> solve_registration(const info_solve_registration &info_registration)
{
    //std::printf("Initial Configuration: %f %f %f\n", info_registration.initial_rotation[0], info_registration.initial_rotation[1], info_registration.initial_rotation[2]);
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    unsigned int numberOfBins = 50;

    metric->SetNumberOfHistogramBins(numberOfBins);

    metric->SetUseMovingImageGradientFilter(true);
    metric->SetUseFixedImageGradientFilter(true);

    using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageType,
                                          ImageType>;

    auto initialTransform = TransformType::New();
    TransformInitializerType::Pointer initializer =
        TransformInitializerType::New();
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(info_registration.fixed_image);
    initializer->SetMovingImage(info_registration.moving_image);
    initializer->InitializeTransform();

    initialTransform->SetRotation(info_registration.initial_rotation[0] * (3.14159265359 / 180), info_registration.initial_rotation[1] * (3.14159265359 / 180), info_registration.initial_rotation[2] * (3.14159265359 / 180));

    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());
    constexpr double translationScale = 1.0 / 1000.0;
    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = translationScale;
    optimizerScales[4] = translationScale;
    optimizerScales[5] = translationScale;
    optimizer->SetScales(optimizerScales);
    optimizer->SetNumberOfIterations(2000);
    optimizer->SetLearningRate(4);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetReturnBestParametersAndValue(true);
    itk::SizeValueType value{10};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(0.9);

    constexpr unsigned int numberOfLevels = 4;

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(numberOfLevels);
    shrinkFactorsPerLevel[0] = 4;
    shrinkFactorsPerLevel[1] = 3;
    shrinkFactorsPerLevel[2] = 2;
    shrinkFactorsPerLevel[3] = 1;

    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(numberOfLevels);
    smoothingSigmasPerLevel[0] = 2;
    smoothingSigmasPerLevel[1] = 1;
    smoothingSigmasPerLevel[2] = 0;
    smoothingSigmasPerLevel[3] = 0;

    registration->SetNumberOfLevels(numberOfLevels);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::RANDOM;

    double samplingPercentage = 0.2;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);

    std::srand(std::time(nullptr)); 
    registration->MetricSamplingReinitializeSeed(std::rand());

    CommandType::Pointer observer = CommandType::New();
    optimizer->AddObserver(itk::StartEvent(), observer);
    optimizer->AddObserver(itk::IterationEvent(), observer);
    optimizer->AddObserver(itk::EndEvent(), observer);
    observer->initialTransform = initialTransform;

    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        throw err;
    }

    TransformType::Pointer finalTransform = TransformType::New();
    auto finalParameters = registration->GetOutput()->Get()->GetParameters();

    finalTransform->SetParameters(finalParameters);
    finalTransform->SetFixedParameters(initialTransform->GetFixedParameters());
    return {optimizer->GetCurrentMetricValue(), finalTransform};
}


ImageType::Pointer manipulate_input_image(ImageType::Pointer image)
{
    using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(image);
    duplicator->Update();

    ImageType::Pointer new_image = duplicator->GetOutput();
    auto origin = image->GetOrigin();

    auto new_origin = origin;
    new_origin[0] += 10.0;
    new_origin[1] += 10.0;
    new_origin[2] += 10.0;

    new_image->SetOrigin(new_origin);

    auto direction = image->GetDirection();

    double euler_vector[3] = {1.0, 1.0, 1.0};

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
    
    /*
      if(argc!=3){
        if(argc>4 || argc == 1){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - input volume, (fixed)\n"
                      << "second parameter - output volume\n" ;
                      return 1;
            }
        if(argc == 2){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - output volume\n";
                      return 1;
        }
    }  
    */
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();

    std::string dirName{argv[1]};
    //std::string dirName{"precious_phantom.mha"};
    fixedImageReader->SetFileName(dirName);

    std::string dirName2{argv[2]};
    //std::string dirName2{"ultrasound_precious_phantom.mha"};
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
    //ImageType::Pointer pointer2movingimage = manipulate_input_image(pointer2fixedimage);
    ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

    auto print_image_info = [](itk::Image<PixelType,3>::Pointer image, std::string name){
        std::cout << "-------------------\n";
        std::cout << "(" << name << ") :\n -------------------\n";
        std::cout << "direction :\n";
        auto direction = image->GetDirection();
        for(size_t i = 0; i < 3; ++i){
            for(size_t j = 0; j < 3; ++j)
                std::cout <<  direction(i,j) << " ";
            std::cout << "\n";
        }
        auto origin = image->GetOrigin();
        std::printf("\norigin: (%f %f %f)",image->GetOrigin()[0],image->GetOrigin()[1],image->GetOrigin()[2]);
        std::printf("\nspacing: (%f %f %f)",image->GetSpacing()[0],image->GetSpacing()[1],image->GetSpacing()[2]);
        std::cout << "\n-------------------\n";
    };

    print_image_info(pointer2fixedimage,"fixed image");
    print_image_info(pointer2movingimage,"moving image");

    std::vector<std::tuple<double, TransformType::Pointer>> full_runs;

    std::array<Eigen::Vector3d,20> initial_configs;
    
    for(auto & vals : initial_configs)
        vals = Eigen::Vector3d::Random()*3;

    {
        std::mutex mut;
        size_t number_of_solved_positions = 0;
        auto pool = curan::utilities::ThreadPool::create(4,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
        std::cout << "starting random registrations!\n";
        for (const auto &initial_config : initial_configs){
            curan::utilities::Job job{"solving registration",[&](){
                {
                    std::lock_guard<std::mutex> g{mut};
                    std::cout << "solving registration...\n";
                }
                auto solution = solve_registration({pointer2fixedimage, pointer2movingimage, initial_config});
                {
                    std::lock_guard<std::mutex> g{mut};
                    full_runs.emplace_back(solution);
                    ++number_of_solved_positions;
                    std::printf("%.2f %% \n",((initial_configs.size()-number_of_solved_positions)/(double)initial_configs.size())*100.0);
                }              
            }};
            pool->submit(job);
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

    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    std::cout << "best estimated transform out of " << initial_configs.size() << " is: \n" << finalTransform;

     using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
 
    auto resample = ResampleFilterType::New();
    resample->SetTransform(finalTransform);
    resample->SetInput(pointer2movingimage);
 
    resample->SetSize(pointer2fixedimage->GetLargestPossibleRegion().GetSize());
    resample->SetOutputOrigin(pointer2fixedimage->GetOrigin());
    resample->SetOutputSpacing(pointer2fixedimage->GetSpacing());
    resample->SetOutputDirection(pointer2fixedimage->GetDirection());
    resample->SetDefaultPixelValue(0);

    using OutputPixelType = unsigned char;
 
    using OutputImageType = itk::Image<OutputPixelType, Dimension>;
 
    using CastFilterType = itk::CastImageFilter<ImageType, itk::Image<unsigned char,3>>;
 
    using WriterType = itk::ImageFileWriter<OutputImageType>;
 
    auto writer = WriterType::New();
    auto caster = CastFilterType::New();
 
    writer->SetFileName(argv[3]);
    //writer->SetFileName("output_test_1.mha");
    
    caster->SetInput(resample->GetOutput());
    writer->SetInput(caster->GetOutput());

    try{
        writer->Update();
    }
    catch (...){
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }    
    return 0;
}