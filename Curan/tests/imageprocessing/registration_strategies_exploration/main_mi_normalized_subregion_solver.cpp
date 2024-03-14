#include "itkMultiResolutionImageRegistrationMethod.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
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
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkNormalVariateGenerator.h"
#include "itkImageRegistrationMethod.h"

#include <optional>
#include <charconv>

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
    using OptimizerType = itk::OnePlusOneEvolutionaryOptimizer;
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
            std::cout << currentValue << "   ";
            std::cout << optimizer->GetFrobeniusNorm() << "   ";
            std::cout << optimizer->GetCurrentPosition() << std::endl;
            m_LastMetricValue = currentValue;
        }
    }

private:
    double m_LastMetricValue;
};

constexpr unsigned int Dimension = 3;
using PixelType = uint16_t;

using FixedImageType = itk::Image<PixelType, Dimension>;
using MovingImageType = itk::Image<PixelType, Dimension>;

std::optional<FixedImageType::Pointer> read_volume_from_directory(const std::string &path)
{
    using ReaderType = itk::ImageSeriesReader<FixedImageType>;
    auto reader = ReaderType::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();
    dicomIO->LoadSequencesOn();
    dicomIO->SetMaxSizeLoadEntry(0xffffff);
    dicomIO->SetKeepOriginalUID(true);
    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    nameGenerator->SetDirectory(path);

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(nameGenerator->GetSeriesUIDs().front());

    if (fileNames.size() == 0)
        return std::nullopt;

    reader->SetFileNames(fileNames);
    try
    {
        reader->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }
    return reader->GetOutput();
}

MovingImageType::Pointer manipulate_input_image(FixedImageType::Pointer image)
{
    using DuplicatorType = itk::ImageDuplicator<FixedImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(image);
    duplicator->Update();

    MovingImageType::Pointer new_image = duplicator->GetOutput();
    auto origin = image->GetOrigin();

    auto new_origin = origin;
    new_origin[0] = 0.0;
    new_origin[1] = 0.0;
    new_origin[2] = 0.0;

    new_image->SetOrigin(origin);

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

    new_image->SetDirection(direction);

    MovingImageType::IndexType start;
    start[0] = 0;
    start[1] = 0;
    start[2] = 0;

    MovingImageType::SizeType end;
    end[0] = std::round(new_image->GetLargestPossibleRegion().GetSize()[0] * 0.5);
    end[1] = std::round(new_image->GetLargestPossibleRegion().GetSize()[1] * 0.5);
    end[2] = std::round(new_image->GetLargestPossibleRegion().GetSize()[2] * 0.5);

    MovingImageType::RegionType region;
    region.SetIndex(start);
    region.SetSize(end);

    using FilterType = itk::RegionOfInterestImageFilter<MovingImageType, MovingImageType>;
    auto filter = FilterType::New();
    filter->SetInput(new_image);
    filter->SetRegionOfInterest(region);

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &error)
    {
        std::cerr << "Error: " << error << std::endl;
        throw std::runtime_error("error");
    }
    return filter->GetOutput();
}

int main(int argc, const char *argv[])
{
    try
    {
        std::optional<FixedImageType::Pointer> possible_image_to_register = read_volume_from_directory(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524");

        if (!possible_image_to_register)
            return EXIT_FAILURE;

        FixedImageType::Pointer image_to_register = *possible_image_to_register;
        MovingImageType::Pointer moving_image_to_register = manipulate_input_image(image_to_register);

        std::printf("size(%llu %llu %llu)\n",
                    moving_image_to_register->GetLargestPossibleRegion().GetSize()[0],
                    moving_image_to_register->GetLargestPossibleRegion().GetSize()[1],
                    moving_image_to_register->GetLargestPossibleRegion().GetSize()[2]);

        std::cout << "both moving and fixed image are defined\n";

        const std::string beforeOutImagefile = "before_manipulated_file.nrrd";
        const std::string outImagefile = "manipulated_file.nrrd";
        const PixelType backgroundGrayLevel = (argc > 3) ? std::stoi(argv[3]) : 100;
        const std::string checkerBoardBefore = (argc > 4) ? argv[4] : "";
        const std::string checkerBoardAfter = (argc > 5) ? argv[5] : "";
        const bool useExplicitPDFderivatives = (argc > 6) ? static_cast<bool>(std::stoi(argv[7])) : true;

        using TransformType = itk::VersorRigid3DTransform<double>;

        using OptimizerType = itk::OnePlusOneEvolutionaryOptimizer;
        using InterpolatorType =
            itk::LinearInterpolateImageFunction<MovingImageType, double>;
        using RegistrationType =
            itk::ImageRegistrationMethod<FixedImageType, MovingImageType>;

        using MetricType =
            itk::NormalizedMutualInformationHistogramImageToImageMetric<
                FixedImageType,
                MovingImageType>;

        auto transform = TransformType::New();
        auto optimizer = OptimizerType::New();
        auto interpolator = InterpolatorType::New();
        auto registration = RegistrationType::New();

        registration->SetOptimizer(optimizer);
        registration->SetTransform(transform);
        registration->SetInterpolator(interpolator);

        auto metric = MetricType::New();
        registration->SetMetric(metric);

        unsigned int numberOfHistogramBins = 32;
        if (argc > 4)
        {
            numberOfHistogramBins = std::stoi(argv[4]);
            std::cout << "Using " << numberOfHistogramBins << " Histogram bins"
                      << std::endl;
        }
        MetricType::HistogramType::SizeType histogramSize;
        histogramSize.SetSize(2);
        histogramSize[0] = numberOfHistogramBins;
        histogramSize[1] = numberOfHistogramBins;
        metric->SetHistogramSize(histogramSize);

        const unsigned int numberOfParameters = transform->GetNumberOfParameters();

        using ScalesType = MetricType::ScalesType;
        ScalesType scales(numberOfParameters);

        scales[0] = 1.0;
        scales[1] = 1.0;
        scales[2] = 1.0;
        scales[3] = 1.0 / 150.0;
        scales[4] = 1.0 / 150.0;
        scales[5] = 1.0 / 150.0;

        metric->SetDerivativeStepLengthScales(scales);

        FixedImageType::ConstPointer fixedImage = *possible_image_to_register;

        registration->SetFixedImageRegion(image_to_register->GetBufferedRegion());

        registration->SetFixedImage(fixedImage);
        registration->SetMovingImage(moving_image_to_register);

        using TransformInitializerType = itk::CenteredTransformInitializer<
            TransformType,
            FixedImageType, MovingImageType>;
        TransformInitializerType::Pointer initializer = TransformInitializerType::New();

        initializer->SetTransform(transform);
        initializer->SetFixedImage(image_to_register);
        initializer->SetMovingImage(moving_image_to_register);

        initializer->MomentsOn();
        initializer->InitializeTransform();

        auto old_transform = transform->Clone();

        using ParametersType = RegistrationType::ParametersType;
        ParametersType initialParameters = transform->GetParameters();

        registration->SetInitialTransformParameters(initialParameters);

        std::cout << "Initial transform parameters = ";
        std::cout << initialParameters << std::endl;

        using OptimizerScalesType = OptimizerType::ScalesType;
        OptimizerScalesType optimizerScales(transform->GetNumberOfParameters());

        FixedImageType::RegionType region = fixedImage->GetLargestPossibleRegion();
        FixedImageType::SizeType size = region.GetSize();
        FixedImageType::SpacingType spacing = fixedImage->GetSpacing();

        optimizerScales[0] = 1.0;
        optimizerScales[1] = 1.0;
        optimizerScales[2] = 1.0;
        optimizerScales[3] = 1.0 / 100.0;
        optimizerScales[4] = 1.0 / 100.0;
        optimizerScales[5] = 1.0 / 100.0;
        optimizer->SetScales(optimizerScales);

        using GeneratorType = itk::Statistics::NormalVariateGenerator;
        auto generator = GeneratorType::New();
        generator->Initialize(13345);

        optimizer->MaximizeOn();
        optimizer->SetNormalVariateGenerator(generator);
        double initialRadius = 0.1;
        optimizer->Initialize(initialRadius);
        double epsilon = 0.001;
        optimizer->SetEpsilon(epsilon);
        optimizer->SetMaximumIteration(2000);

        // Create the Command observer and register it with the optimizer.
        //
        auto observer = CommandIterationUpdate::New();
        optimizer->AddObserver(itk::IterationEvent(), observer);
        try
        {
            registration->Update();
            std::cout << "Optimizer stop condition: "
                      << registration->GetOptimizer()->GetStopConditionDescription()
                      << std::endl;
        }
        catch (const itk::ExceptionObject &err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            return EXIT_FAILURE;
        }

        // Software Guide : EndCodeSnippet
        ParametersType finalParameters = registration->GetLastTransformParameters();
        double versor1 = finalParameters[0];
        double versor2 = finalParameters[1];
        double versor3 = finalParameters[2];
        double TranslationAlongX = finalParameters[3];
        double TranslationAlongY = finalParameters[4];
        double TranslationAlongZ = finalParameters[5];
        unsigned int numberOfIterations = optimizer->GetCurrentIteration();
        double bestValue = optimizer->GetValue();

        std::printf("\nTranslation (%f %f %f) \nRotation (%f %f %f) \nIterations (%d) \nBest Value (%f)\n",
                    TranslationAlongX,
                    TranslationAlongY,
                    TranslationAlongZ,
                    versor1,
                    versor2,
                    versor3,
                    numberOfIterations,
                    bestValue);
        using InternalPixelType = float;
        using InternalImageType = itk::Image<InternalPixelType, Dimension>;
        using FixedCastFilterType = itk::CastImageFilter<FixedImageType, InternalImageType>;
        using MovingCastFilterType = itk::CastImageFilter<MovingImageType, InternalImageType>;
        FixedCastFilterType::Pointer fixedCaster = FixedCastFilterType::New();
        MovingCastFilterType::Pointer movingCaster = MovingCastFilterType::New();
        fixedCaster->SetInput(image_to_register);
        movingCaster->SetInput(moving_image_to_register);

        fixedCaster->Update();
        movingCaster->Update();

        using ResampleFilterType = itk::ResampleImageFilter<
            InternalImageType,
            InternalImageType>;
        TransformType::Pointer finalTransform = TransformType::New();
        finalTransform->SetParameters(finalParameters);
        finalTransform->SetFixedParameters(transform->GetFixedParameters());

        {
            ResampleFilterType::Pointer resampler = ResampleFilterType::New();
            resampler->SetInput(movingCaster->GetOutput());

            resampler->SetTransform(old_transform);
            FixedImageType::Pointer fixedImage = image_to_register;
            resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
            resampler->SetOutputOrigin(fixedImage->GetOrigin());
            resampler->SetOutputSpacing(fixedImage->GetSpacing());
            resampler->SetOutputDirection(fixedImage->GetDirection());
            resampler->SetDefaultPixelValue(0.0);

            using DifferenceFilterType = itk::SubtractImageFilter<InternalImageType, InternalImageType, InternalImageType>;
            DifferenceFilterType::Pointer difference = DifferenceFilterType::New();

            difference->SetInput1(fixedCaster->GetOutput());
            difference->SetInput2(resampler->GetOutput());

            using RescalerType = itk::RescaleIntensityImageFilter<InternalImageType, MovingImageType>;
            RescalerType::Pointer intensityRescaler = RescalerType::New();
            intensityRescaler->SetInput(difference->GetOutput());
            intensityRescaler->SetOutputMinimum(0);
            intensityRescaler->SetOutputMaximum(255);

            using WriterType = itk::ImageFileWriter<MovingImageType>;

            WriterType::Pointer writer = WriterType::New();
            writer->SetInput(intensityRescaler->GetOutput());
            writer->SetFileName(beforeOutImagefile);

            writer->Update();
        }

        {
            ResampleFilterType::Pointer resampler = ResampleFilterType::New();
            resampler->SetInput(movingCaster->GetOutput());

            resampler->SetTransform(finalTransform);
            FixedImageType::Pointer fixedImage = image_to_register;
            resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
            resampler->SetOutputOrigin(fixedImage->GetOrigin());
            resampler->SetOutputSpacing(fixedImage->GetSpacing());
            resampler->SetOutputDirection(fixedImage->GetDirection());
            resampler->SetDefaultPixelValue(0.0);

            using DifferenceFilterType = itk::SubtractImageFilter<InternalImageType, InternalImageType, InternalImageType>;
            DifferenceFilterType::Pointer difference = DifferenceFilterType::New();

            difference->SetInput1(fixedCaster->GetOutput());
            difference->SetInput2(resampler->GetOutput());

            using RescalerType = itk::RescaleIntensityImageFilter<InternalImageType, MovingImageType>;
            RescalerType::Pointer intensityRescaler = RescalerType::New();
            intensityRescaler->SetInput(difference->GetOutput());
            intensityRescaler->SetOutputMinimum(0);
            intensityRescaler->SetOutputMaximum(255);

            using WriterType = itk::ImageFileWriter<MovingImageType>;

            WriterType::Pointer writer = WriterType::New();
            writer->SetInput(intensityRescaler->GetOutput());
            writer->SetFileName(outImagefile);

            writer->Update();
        }

        std::cout << "Initial Transform: \n"
                  << old_transform << std::endl;
        std::cout << "Final Transform: \n"
                  << finalTransform << std::endl;

        return EXIT_SUCCESS;
    }
    catch (const itk::ExceptionObject &excp)
    {
        std::cerr << "Problem solving registration= " << std::endl;
        std::cerr << argv[0] << std::endl;
        std::cerr << excp << std::endl;
        return EXIT_FAILURE;
    }
}
