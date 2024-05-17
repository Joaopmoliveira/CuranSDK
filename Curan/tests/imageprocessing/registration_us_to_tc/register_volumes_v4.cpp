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
    std::printf("Initial Configuration: %f %f %f\n", info_registration.initial_rotation[0], info_registration.initial_rotation[1], info_registration.initial_rotation[2]);
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    unsigned int numberOfBins = 50;

    metric->SetNumberOfHistogramBins(numberOfBins);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);

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
    optimizer->SetRelaxationFactor(0.8);

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

    double samplingPercentage = 0.01;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);
    registration->MetricSamplingReinitializeSeed(121213);

    ImageType::RegionType region = info_registration.fixed_image->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();
    ImageType::SpacingType spacing = info_registration.fixed_image->GetSpacing();

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

    std::vector<std::tuple<double, TransformType::Pointer>> full_runs;

    std::array<Eigen::Vector3d,20> initial_configs;
    for(auto & vals : initial_configs)
        vals = Eigen::Vector3d::Random();

    auto pool = curan::utilities::ThreadPool::create(8);
    std::mutex mut;
    size_t number_of_threads_running = 0;
    std::condition_variable cv;
    {
        std::cout << "starting random registrations!\n";
        for (const auto &initial_config : initial_configs){
            {
                std::lock_guard<std::mutex> g{mut};
                
                std::cout << "submiting async work to thread pool\n";
            }
            curan::utilities::Job job{"solving registration",[&](){
                {
                    std::lock_guard<std::mutex> g{mut};
                    ++number_of_threads_running;
                }
                auto solution = solve_registration({pointer2fixedimage, pointer2movingimage, initial_config});
                
                {
                    std::lock_guard<std::mutex> g{mut};
                    full_runs.emplace_back(solution);
                    --number_of_threads_running;
                }
                cv.notify_one();
                std::cout << "finished!\n";
            }};
            pool->submit(job);
        }
        {
            std::lock_guard<std::mutex> g{mut};
            std::cout << "waiting for thread pool to stop\n";
        }
    }
    std::unique_lock lk(mut);
    cv.wait(lk, [&]{ return number_of_threads_running==0; });
    std::cout << "destroyed thread pool\n";

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = std::numeric_limits<double>::max();
    for (const auto &possible : full_runs){
        if (minimum_val > std::get<0>(possible)){
            minimum_index = current_index;
            minimum_val = std::get<0>(possible);
        }
        ++current_index;
    }

    auto finalTransform = std::get<1>(full_runs[minimum_index]);

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