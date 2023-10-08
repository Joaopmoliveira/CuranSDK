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
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include <iostream>
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "utils/TheadPool.h"
#include "rendering/Volume.h"

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

void updateBaseTexture3D(vsg::floatArray3D &image, ImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    ImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<ImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        ImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

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
    curan::renderable::Volume *moving_pointer_to_volume = nullptr;
    Eigen::Matrix<double, 4, 4> moving_homogenenous;
    TransformType::Pointer initialTransform;

    void set_pointer(curan::renderable::Volume *in_moving_pointer_to_volume)
    {
        moving_pointer_to_volume = in_moving_pointer_to_volume;
    }

    void Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }
    void Execute(const itk::Object *object, const itk::EventObject &event) override
    {
        auto optimizer = static_cast<OptimizerPointer>(object);
        TransformType::Pointer transform = TransformType::New();
        auto vals = optimizer->GetCurrentPosition();
        // std::cout << vals << "\n";
        transform->SetParameters(optimizer->GetCurrentPosition());
        transform->SetFixedParameters(initialTransform->GetFixedParameters());
        TransformType::MatrixType matrix = transform->GetMatrix();
        TransformType::OffsetType offset = transform->GetOffset();

        Eigen::Matrix<double, 4, 4> estimated_unknown_transformation = Eigen::Matrix<double, 4, 4>::Identity();

        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                estimated_unknown_transformation(row, col) = matrix(row, col);

        estimated_unknown_transformation(0, 3) = offset[0];
        estimated_unknown_transformation(1, 3) = offset[1];
        estimated_unknown_transformation(2, 3) = offset[2];

        auto estimated_overlap = estimated_unknown_transformation.inverse() * moving_homogenenous;

        auto moving_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);

        for (size_t row = 0; row < 4; ++row)
            for (size_t col = 0; col < 4; ++col)
                moving_homogenenous_transformation(col, row) = estimated_overlap(row, col);

        moving_homogenenous_transformation(3, 0) *= 1e-3;
        moving_homogenenous_transformation(3, 1) *= 1e-3;
        moving_homogenenous_transformation(3, 2) *= 1e-3;

        if (moving_pointer_to_volume)
            moving_pointer_to_volume->update_transform(moving_homogenenous_transformation);
    }
};

struct CurrentInitialPose {
    Eigen::Matrix<double, 4, 4> mat_moving_here = Eigen::Matrix<double, 4, 4>::Identity();
    std::mutex mut;

    void update_matrix(const Eigen::Matrix<double, 4, 4>& supply){
        std::lock_guard<std::mutex> g{mut};
        mat_moving_here = supply;
    }

    Eigen::Matrix<double,4,4> get_matrix(){
        std::lock_guard<std::mutex> g{mut};
        return mat_moving_here;
    }
};

struct info_solve_registration{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    curan::renderable::Volume *volume_moving;
    CurrentInitialPose& moving_homogenenous;
    std::atomic<bool>& optimization_running;
    std::atomic<bool>& robot_client_commands_volume_init;
    std::shared_ptr<curan::utilities::ThreadPool> thread_pool;
    std::vector<std::tuple<double,TransformType::Pointer>>& full_runs;
    std::unique_ptr<kuka::Robot> robot; 
    std::unique_ptr<RobotParameters> iiwa;
    vsg::ref_ptr<curan::renderable::Renderable> robot_render;
};

std::tuple<double, TransformType::Pointer> solve_registration(info_solve_registration &info_registration)
{
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

    registration->InPlaceOn();
    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());
    constexpr double translationScale = 1.0 / 100.0;
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
    observer->moving_homogenenous = info_registration.moving_homogenenous.get_matrix();
    observer->initialTransform = initialTransform;
    observer->moving_pointer_to_volume = info_registration.volume_moving;
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

    return {optimizer->GetCurrentMetricValue(), initialTransform};
}