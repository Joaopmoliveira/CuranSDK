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
using InterpolatorType = itk::LinearInterpolateImageFunction<
            ImageType,
            double>;


class CommandIterationUpdate : public itk::Command
{
public:
    using Self = CommandIterationUpdate;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandIterationUpdate() { m_LastMetricValue = 0; }

public:
    using OptimizerPointer = const OptimizerType *;

    void
    Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }

    void
    Execute(const itk::Object *object, const itk::EventObject &event) override
    {
        auto optimizer = static_cast<OptimizerPointer>(object);
        if (!itk::IterationEvent().CheckEvent(&event))
        {
            return;
        }
        double currentValue = optimizer->GetValue();
        // Only print out when the Metric value changes
        if (itk::Math::abs(m_LastMetricValue - currentValue) > 1e-7)
        {
            std::cout << optimizer->GetCurrentIteration() << "   ";
            std::cout << currentValue << std::endl;
            m_LastMetricValue = currentValue;
        }
    }

private:
    double m_LastMetricValue;
};



struct info_solve_registration
{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    const Eigen::Vector3d &initial_rotation;
};

std::tuple<std::optional<double>,TransformType::Pointer,TransformType::Pointer> solve_registration(const info_solve_registration &info_registration)
{
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    unsigned int numberOfBins = 30;

    metric->SetNumberOfHistogramBins(numberOfBins);

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
    constexpr double translationScale = 1.0 / 1000.0;
    optimizerScales[0] = 5.0;
    optimizerScales[1] = 5.0;
    optimizerScales[2] = 5.0;
    optimizerScales[3] = translationScale;
    optimizerScales[4] = translationScale;
    optimizerScales[5] = translationScale;
    optimizer->SetScales(optimizerScales);
    optimizer->SetNumberOfIterations(10);
    optimizer->SetLearningRate(8);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{10};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(0.7);
    //auto observer = CommandIterationUpdate::New();
    //optimizer->AddObserver(itk::IterationEvent(), observer);

    constexpr unsigned int numberOfLevels = 1;

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(numberOfLevels);
    shrinkFactorsPerLevel[0] = 1;
    //shrinkFactorsPerLevel[1] = 3;
    //shrinkFactorsPerLevel[2] = 2;
    //shrinkFactorsPerLevel[3] = 1;

    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(numberOfLevels);
    smoothingSigmasPerLevel[0] = 0;
    //smoothingSigmasPerLevel[1] = 1;
    //smoothingSigmasPerLevel[2] = 0;
    //smoothingSigmasPerLevel[3] = 0;

    registration->SetNumberOfLevels(numberOfLevels);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::RANDOM;

    double samplingPercentage = 0.4;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);

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

    std::vector<std::tuple<std::optional<double>, TransformType::Pointer,TransformType::Pointer>> full_runs;

    std::array<Eigen::Vector3d,125> initial_configs;
    
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
        

    {
        std::mutex mut;
        size_t number_of_solved_positions = 0;
        auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
        std::cout << "starting random registrations!\n";
        for (const auto &initial_config : initial_configs){
            curan::utilities::Job job{"solving registration",[&](){
                auto solution = solve_registration({pointer2fixedimage, pointer2movingimage, initial_config});
                {
                    std::lock_guard<std::mutex> g{mut};
                    if(std::get<0>(solution))
                        std::printf("cost: %.2f %.2f %% \n",*std::get<0>(solution),100.0-((initial_configs.size()-number_of_solved_positions)/(double)initial_configs.size())*100.0);
                    else
                        std::printf("%.2f %% \n",((initial_configs.size()-number_of_solved_positions)/(double)initial_configs.size())*100.0);
                    full_runs.emplace_back(solution);
                    ++number_of_solved_positions;

                   
                }              
            }};
            pool->submit(job);
        } 
    }

    size_t minimum_index = std::numeric_limits<size_t>::max();
    size_t current_index = 0;
    double minimum_val = std::numeric_limits<double>::max();
    for (const auto &possible_best_solution : full_runs){
        if (std::get<0>(possible_best_solution) && minimum_val > *std::get<0>(possible_best_solution)){
            minimum_index = current_index;
            minimum_val = *std::get<0>(possible_best_solution);
        }
        ++current_index;
    }

    if(minimum_index==std::numeric_limits<size_t>::max()){
        std::cout << "No solution found minimizes the error between the two images\n";
        return 1;
    }

    std::printf("Choosen cost: %.2f\n",std::get<0>(full_runs[minimum_index]));
    
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

    return 0;
}