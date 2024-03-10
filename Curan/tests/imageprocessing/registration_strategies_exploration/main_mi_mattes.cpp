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
#include "itkGDCMSeriesFileNames.h"

#include <optional>
#include <charconv>

template <typename TRegistration>
class RegistrationInterfaceCommand : public itk::Command
{
public:
    using Self = RegistrationInterfaceCommand;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    RegistrationInterfaceCommand(){};

public:
    using RegistrationType = TRegistration;
    using RegistrationPointer = RegistrationType *;
    using OptimizerType = itk::VersorRigid3DTransformOptimizer;
    using OptimizerPointer = OptimizerType *;
    void Execute(itk::Object *object, const itk::EventObject &event)
    {
        if (!(itk::IterationEvent().CheckEvent(&event)))
        {
            return;
        }
        RegistrationPointer registration = static_cast<RegistrationPointer>(object);
        OptimizerPointer optimizer = static_cast<OptimizerPointer>(registration->GetModifiableOptimizer());
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "MultiResolution Level : "
                  << registration->GetCurrentLevel() << std::endl;
        std::cout << std::endl;
        if (registration->GetCurrentLevel() == 0)
        {
            optimizer->SetMaximumStepLength(16.00);
            optimizer->SetMinimumStepLength(0.01);
        }
        else
        {
            optimizer->SetMaximumStepLength(
                optimizer->GetMaximumStepLength() * 0.25);
            optimizer->SetMinimumStepLength(
                optimizer->GetMinimumStepLength() * 0.1);
        }
    }
    void Execute(const itk::Object *, const itk::EventObject &)
    {
        return;
    }
};

class CommandIterationUpdate : public itk::Command
{
public:
    using Self = CommandIterationUpdate;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandIterationUpdate(){};

public:
    using OptimizerType = itk::VersorRigid3DTransformOptimizer;
    using OptimizerPointer = const OptimizerType *;
    void Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }
    void Execute(const itk::Object *object, const itk::EventObject &event) override
    {
        OptimizerPointer optimizer = static_cast<OptimizerPointer>(object);
        if (!(itk::IterationEvent().CheckEvent(&event)))
        {
            return;
        }
        std::cout << optimizer->GetCurrentIteration() << "   ";
        std::cout << optimizer->GetValue() << "   ";
        std::cout << optimizer->GetCurrentPosition() << std::endl;
    }
};

constexpr unsigned int Dimension = 3;
using PixelType = unsigned short;

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

    using DictionaryType = itk::MetaDataDictionary;

    const DictionaryType &dictionary = dicomIO->GetMetaDataDictionary();

    using MetaDataStringType = itk::MetaDataObject<std::string>;

    auto itr = dictionary.Begin();
    auto end = dictionary.End();

    auto query = [&](const std::string &entryID)
    {
        auto tagItr = dictionary.Find(entryID);
        std::optional<std::string> tagvalue = std::nullopt;
        if (tagItr == end)
        {
            std::cout << "Tag " << entryID;
            std::cout << " not found in the DICOM header" << std::endl;
            return tagvalue;
        }
        MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType *>(tagItr->second.GetPointer());

        if (entryvalue)
        {
            tagvalue = entryvalue->GetMetaDataObjectValue();
            std::cout << "Patient's (" << entryID << ") ";
            std::cout << " is: " << *tagvalue << std::endl;
        }
        else
            std::cout << "Entry was not of string type" << std::endl;
        return tagvalue;
    };

    auto bits_stored_atribute = query("0028|0101"); // Bits Stored Attribute

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<FixedImageType, FixedImageType>;
    auto shiftFilter = ShiftScaleFilterType::New();

    auto compute_conversion = [](const std::string &number_in_string)
    {
        std::optional<int> result = std::nullopt;
        int result_l{};
        auto [ptr, ec] = std::from_chars(number_in_string.data(), number_in_string.data() + number_in_string.size(), result_l);

        if (ec == std::errc())
            result = result_l;

        return result;
    };

    int bits_stored = bits_stored_atribute ? (compute_conversion(*bits_stored_atribute) ? *compute_conversion(*bits_stored_atribute) : 16) : 16; // Bits Allocated Attribute

    std::printf("scalling factor is: (%f) with bits stored (%d)\n", std::pow(2, sizeof(PixelType) * 8.0 - bits_stored), bits_stored);

    shiftFilter->SetScale(std::pow(2, sizeof(PixelType) * 8.0 - bits_stored));
    shiftFilter->SetShift(0);
    shiftFilter->SetInput(reader->GetOutput());

    using FilterType = itk::CastImageFilter<FixedImageType, FixedImageType>;
    auto filter = FilterType::New();
    filter->SetInput(shiftFilter->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }

    return filter->GetOutput();
}

int main(int argc, const char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Missing Parameters " << std::endl;
        std::cerr << "Usage: " << argv[0];
        std::cerr << " fixedImageFile  movingImageFile ";
        std::cerr << " outputImagefile [backgroundGrayLevel]";
        std::cerr << " [checkerBoardBefore] [checkerBoardAfter]";
        std::cerr << " [useExplicitPDFderivatives ] " << std::endl;
        std::cerr << " [numberOfBins] [numberOfSamples ] " << std::endl;
        return EXIT_FAILURE;
    }
    const std::string fixedImageFile = argv[1];
    const std::string movingImageFile = argv[2];
    const std::string outImagefile = argv[3];
    const PixelType backgroundGrayLevel = (argc > 4) ? std::stoi(argv[4]) : 100;
    const std::string checkerBoardBefore = (argc > 5) ? argv[5] : "";
    const std::string checkerBoardAfter = (argc > 6) ? argv[6] : "";
    const bool useExplicitPDFderivatives = (argc > 7) ? static_cast<bool>(std::stoi(argv[7])) : false;
    const int numberOfBins = (argc > 8) ? std::stoi(argv[8]) : 0;
    const int numberOfSamples = (argc > 9) ? std::stoi(argv[9]) : 0;
    using InternalPixelType = float;
    using InternalImageType = itk::Image<InternalPixelType, Dimension>;
    using TransformType = itk::VersorRigid3DTransform< double >;
    using OptimizerType = itk::VersorRigid3DTransformOptimizer;
    using InterpolatorType = itk::LinearInterpolateImageFunction<
        InternalImageType,
        double>;
    using MetricType = itk::MattesMutualInformationImageToImageMetric<
        InternalImageType,
        InternalImageType>;
    using RegistrationType = itk::MultiResolutionImageRegistrationMethod<
        InternalImageType,
        InternalImageType>;
    using FixedImagePyramidType = itk::MultiResolutionPyramidImageFilter<
        InternalImageType, InternalImageType>;
    using MovingImagePyramidType = itk::MultiResolutionPyramidImageFilter<
        InternalImageType, InternalImageType>;
    TransformType::Pointer transform = TransformType::New();
    OptimizerType::Pointer optimizer = OptimizerType::New();
    InterpolatorType::Pointer interpolator = InterpolatorType::New();
    RegistrationType::Pointer registration = RegistrationType::New();
    MetricType::Pointer metric = MetricType::New();
    FixedImagePyramidType::Pointer fixedImagePyramid =
        FixedImagePyramidType::New();
    MovingImagePyramidType::Pointer movingImagePyramid =
        MovingImagePyramidType::New();
    registration->SetOptimizer(optimizer);
    registration->SetTransform(transform);
    registration->SetInterpolator(interpolator);
    registration->SetMetric(metric);
    registration->SetFixedImagePyramid(fixedImagePyramid);
    registration->SetMovingImagePyramid(movingImagePyramid);
    using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
    using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
    FixedImageReaderType::Pointer fixedImageReader = FixedImageReaderType::New();
    MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();
    fixedImageReader->SetFileName(fixedImageFile);
    movingImageReader->SetFileName(movingImageFile);

    using FixedCastFilterType = itk::CastImageFilter<
        FixedImageType, InternalImageType>;
    using MovingCastFilterType = itk::CastImageFilter<
        MovingImageType, InternalImageType>;
    FixedCastFilterType::Pointer fixedCaster = FixedCastFilterType::New();
    MovingCastFilterType::Pointer movingCaster = MovingCastFilterType::New();
    fixedCaster->SetInput(fixedImageReader->GetOutput());
    movingCaster->SetInput(movingImageReader->GetOutput());
    registration->SetFixedImage(fixedCaster->GetOutput());
    registration->SetMovingImage(movingCaster->GetOutput());
    // Software Guide : EndCodeSnippet
    fixedCaster->Update();
    registration->SetFixedImageRegion(
        fixedCaster->GetOutput()->GetBufferedRegion());
    using ParametersType = RegistrationType::ParametersType;
    ParametersType initialParameters(transform->GetNumberOfParameters());
    initialParameters[0] = 0.0; // Initial offset in mm along X
    initialParameters[1] = 0.0; // Initial offset in mm along Y
    registration->SetInitialTransformParameters(initialParameters);
    metric->SetNumberOfHistogramBins(128);
    metric->SetNumberOfSpatialSamples(50000);
    if (argc > 8)
    {
        // optionally, override the values with numbers taken from the command line arguments.
        metric->SetNumberOfHistogramBins(numberOfBins);
    }
    if (argc > 9)
    {
        // optionally, override the values with numbers taken from the command line arguments.
        metric->SetNumberOfSpatialSamples(numberOfSamples);
    }
    metric->ReinitializeSeed(76926294);
    // Software Guide : EndCodeSnippet
    if (argc > 7)
    {
        // Define whether to calculate the metric derivative by explicitly
        // computing the derivatives of the joint PDF with respect to the Transform
        // parameters, or doing it by progressively accumulating contributions from
        // each bin in the joint PDF.
        metric->SetUseExplicitPDFDerivatives(useExplicitPDFderivatives);
    }
    optimizer->SetNumberOfIterations(200);
    optimizer->SetRelaxationFactor(0.9);
    CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
    optimizer->AddObserver(itk::IterationEvent(), observer);
    using CommandType = RegistrationInterfaceCommand<RegistrationType>;
    CommandType::Pointer command = CommandType::New();
    registration->AddObserver(itk::IterationEvent(), command);
    registration->SetNumberOfLevels(3);
    try
    {
        registration->Update();
        std::cout << "Optimizer stop condition: "
                  << registration->GetOptimizer()->GetStopConditionDescription()
                  << std::endl;
    }
    catch (itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    // Software Guide : EndCodeSnippet
    ParametersType finalParameters = registration->GetLastTransformParameters();
    double TranslationAlongX = finalParameters[0];
    double TranslationAlongY = finalParameters[1];
    unsigned int numberOfIterations = optimizer->GetCurrentIteration();
    double bestValue = optimizer->GetValue();
    // Print out results
    //
    std::cout << "Result = " << std::endl;
    std::cout << " Translation X = " << TranslationAlongX << std::endl;
    std::cout << " Translation Y = " << TranslationAlongY << std::endl;
    std::cout << " Iterations    = " << numberOfIterations << std::endl;
    std::cout << " Metric value  = " << bestValue << std::endl;
    using ResampleFilterType = itk::ResampleImageFilter<
        MovingImageType,
        FixedImageType>;
    TransformType::Pointer finalTransform = TransformType::New();
    finalTransform->SetParameters(finalParameters);
    finalTransform->SetFixedParameters(transform->GetFixedParameters());
    ResampleFilterType::Pointer resample = ResampleFilterType::New();
    resample->SetTransform(finalTransform);
    resample->SetInput(movingImageReader->GetOutput());
    FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();
    resample->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
    resample->SetOutputOrigin(fixedImage->GetOrigin());
    resample->SetOutputSpacing(fixedImage->GetSpacing());
    resample->SetOutputDirection(fixedImage->GetDirection());
    resample->SetDefaultPixelValue(backgroundGrayLevel);
    using OutputPixelType = unsigned char;
    using OutputImageType = itk::Image<OutputPixelType, Dimension>;
    using CastFilterType = itk::CastImageFilter<
        FixedImageType,
        OutputImageType>;
    using WriterType = itk::ImageFileWriter<OutputImageType>;
    WriterType::Pointer writer = WriterType::New();
    CastFilterType::Pointer caster = CastFilterType::New();
    writer->SetFileName(outImagefile);
    caster->SetInput(resample->GetOutput());
    writer->SetInput(caster->GetOutput());
    writer->Update();
    using CheckerBoardFilterType = itk::CheckerBoardImageFilter<FixedImageType>;
    CheckerBoardFilterType::Pointer checker = CheckerBoardFilterType::New();
    checker->SetInput1(fixedImage);
    checker->SetInput2(resample->GetOutput());
    caster->SetInput(checker->GetOutput());
    writer->SetInput(caster->GetOutput());
    resample->SetDefaultPixelValue(0);
    // Before registration
    TransformType::Pointer identityTransform = TransformType::New();
    identityTransform->SetIdentity();
    resample->SetTransform(identityTransform);
    for (int q = 0; q < argc; ++q)
    {
        std::cout << q << " " << argv[q] << std::endl;
    }
    if (checkerBoardBefore != std::string(""))
    {
        writer->SetFileName(checkerBoardBefore);
        writer->Update();
    }
    // After registration
    resample->SetTransform(finalTransform);
    if (checkerBoardAfter != std::string(""))
    {
        writer->SetFileName(checkerBoardAfter);
        writer->Update();
    }
    return EXIT_SUCCESS;
}
