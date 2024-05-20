#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageRegistrationMethod.h"
#include "itkCenteredTransformInitializer.h"
#include "itkAffineTransform.h"
#include "itkResampleImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkMultiResolutionImageRegistrationMethod.h"


const unsigned int Dimension = 3;
using PixelType = float;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::AffineTransform<double, Dimension>;
using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType = itk::MattesMutualInformationImageToImageMetric<ImageType, ImageType>;
using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
using RegistrationType = itk::ImageRegistrationMethod<ImageType, ImageType>;
using ReaderType = itk::ImageFileReader<ImageType>;
using WriterType = itk::ImageFileWriter<ImageType>;
using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " fixedImageFile movingImageFile outputImageFile" << std::endl;
        return EXIT_FAILURE;
    }

    auto fixedImageReader = ReaderType::New();
    auto movingImageReader = ReaderType::New();
    fixedImageReader->SetFileName(argv[1]);
    movingImageReader->SetFileName(argv[2]);

    auto metric = MetricType::New();
    metric->SetNumberOfHistogramBins(50);
    metric->SetNumberOfSpatialSamples(10000);
    metric->ReinitializeSeed(12345);

    auto optimizer = OptimizerType::New();
    //optimizer->SetMaximumStepLength(4.00);

    optimizer->SetLearningRate(4);
 optimizer->SetRelaxationFactor(0.5);

    optimizer->SetMinimumStepLength(0.01);
    optimizer->SetNumberOfIterations(200);

    auto transform = TransformType::New();
    auto registration = RegistrationType::New();
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
    registration->SetFixedImage(fixedImageReader->GetOutput());
    registration->SetMovingImage(movingImageReader->GetOutput());
    registration->SetTransform(transform);

    auto interpolator = InterpolatorType::New();
    registration->SetInterpolator(interpolator);

    auto initializer = itk::CenteredTransformInitializer<TransformType, ImageType, ImageType>::New();
    initializer->SetTransform(transform);
    initializer->SetFixedImage(fixedImageReader->GetOutput());
    initializer->SetMovingImage(movingImageReader->GetOutput());
    initializer->MomentsOn();
    initializer->InitializeTransform();

    try
    {
        registration->Update();
        std::cout << "Optimizer stop condition: " << registration->GetOptimizer()->GetStopConditionDescription() << std::endl;
    }
    catch (itk::ExceptionObject & err)
    {
        std::cerr << "ExceptionObject caught !" << std::endl;
        std::cerr << err << std::endl;
        return EXIT_FAILURE;
    }

    auto resample = ResampleFilterType::New();
    resample->SetTransform(registration->GetTransform());
    resample->SetInput(movingImageReader->GetOutput());
    resample->SetSize(fixedImageReader->GetOutput()->GetLargestPossibleRegion().GetSize());
    resample->SetOutputOrigin(fixedImageReader->GetOutput()->GetOrigin());
    resample->SetOutputSpacing(fixedImageReader->GetOutput()->GetSpacing());
    resample->SetOutputDirection(fixedImageReader->GetOutput()->GetDirection());
    resample->SetDefaultPixelValue(100);

    auto writer = WriterType::New();
    writer->SetFileName(argv[3]);
    writer->SetInput(resample->GetOutput());

    try
    {
        writer->Update();
    }
    catch (itk::ExceptionObject & ex)
    {
        std::cerr << "ExceptionObject caught !" << std::endl;
        std::cerr << ex << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}



#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
//#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkCommand.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"

class CommandIterationUpdate : public itk::Command
{
public:
    using Self = CommandIterationUpdate;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandIterationUpdate(){};
    CommandIterationUpdate() = default;

public:
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
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
using PixelType = float;
using ImageType = itk::Image<PixelType, Dimension>;
//using TransformType = itk::TranslationTransform<double, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;

using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
//using MetricType =
//itk::MeanSquaresImageToImageMetricv4<ImageType, ImageType>;
using MetricType =
itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::
ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;
using TransformInitializerType =
itk::CenteredTransformInitializer<TransformType,ImageType,ImageType>;

int RegisterImages(const std::string& fixedImagePath, const std::string& movingImagePath) {
    // Readers
    auto fixedImageReader = itk::ImageFileReader<ImageType>::New();
    fixedImageReader->SetFileName(fixedImagePath);
    auto movingImageReader = itk::ImageFileReader<ImageType>::New();
    movingImageReader->SetFileName(movingImagePath);

    try {
        fixedImageReader->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    try {
        movingImageReader->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    auto fixedInterpolator = InterpolatorType::New();
    auto movingInterpolator = InterpolatorType::New();

    unsigned int numberOfBins = 24;
    metric->SetNumberOfHistogramBins(numberOfBins);
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    metric->SetFixedInterpolator(fixedInterpolator);
    metric->SetMovingInterpolator(movingInterpolator);
    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
    RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    double samplingPercentage = 0.20;


    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);
    registration->MetricSamplingReinitializeSeed(121213);
    registration->SetFixedImage(fixedImageReader->GetOutput());
    registration->SetMovingImage(movingImageReader->GetOutput());

/*
    auto movingInitialTransform = TransformType::New();
    TransformInitializerType::Pointer initializer =
TransformInitializerType::New();
    TransformType::ParametersType initialParameters(movingInitialTransform->GetNumberOfParameters());
    initialParameters[0] = 0.0; // Initial offset in mm along X
    initialParameters[1] = 0.0; // Initial offset in mm along Y
    movingInitialTransform->SetParameters(initialParameters);
    registration->SetMovingInitialTransform(movingInitialTransform);

    TransformType::Pointer identityTransform = TransformType::New();
    identityTransform->SetIdentity();
    registration->SetFixedInitialTransform(identityTransform);
*/
    auto initialTransform = TransformType::New();
    auto initializer =TransformInitializerType::New();
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(fixedImageReader->GetOutput());
    initializer->SetMovingImage(movingImageReader->GetOutput());
    initializer->MomentsOn();
    initializer->InitializeTransform();

    using VersorType = TransformType::VersorType;
    using VectorType = VersorType::VectorType;
    VersorType rotation;
    VectorType axis;
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 1.0;
    constexpr double angle = 0;
    rotation.Set(axis, angle);
    initialTransform->SetRotation(rotation);

    registration->SetInitialTransform(initialTransform);



    CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
    optimizer->AddObserver(itk::IterationEvent(), observer);
    optimizer->SetLearningRate(8.00);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetNumberOfIterations(200);
    optimizer->ReturnBestParametersAndValueOn();
    optimizer->SetRelaxationFactor(0.8);


    constexpr unsigned int numberOfLevels = 1;
    
    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(1);
    shrinkFactorsPerLevel[0] = 1;
    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(1);
    smoothingSigmasPerLevel[0] = 0;
    registration->SetNumberOfLevels(numberOfLevels);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    try {
         registration->Update();
        std::cout << "Optimizer stop condition: "
        << registration->GetOptimizer()->GetStopConditionDescription()
        << std::endl;
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    TransformType::ConstPointer transform = registration->GetTransform();

    TransformType::ParametersType finalParameters = transform->GetParameters();

    const double TranslationAlongX = finalParameters[0];
    const double TranslationAlongY = finalParameters[1];
    const unsigned int numberOfIterations = optimizer->GetCurrentIteration();
    const double bestValue = optimizer->GetValue();

    using CompositeTransformType = itk::CompositeTransform<double, Dimension>;
    auto outputCompositeTransform = CompositeTransformType::New();
    outputCompositeTransform->AddTransform(movingInitialTransform);
    outputCompositeTransform->AddTransform(registration->GetModifiableTransform());

    TransformType::MatrixType matrix = finalTransform->GetMatrix();
    TransformType::OffsetType offset = finalTransform->GetOffset();
    std::cout << "Matrix = " << std::endl << matrix << std::endl;
    std::cout << "Offset = " << std::endl << offset << std::endl;



    using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    auto resampler = ResampleFilterType::New();
    resampler->SetInput(movingImageReader->GetOutput());
    resampler->SetTransform(outputCompositeTransform);

    ImageType::Pointer fixedImage = fixedImageReader->GetOutput();
    resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
    resampler->SetOutputOrigin(fixedImage->GetOrigin());
    resampler->SetOutputSpacing(fixedImage->GetSpacing());
    resampler->SetOutputDirection(fixedImage->GetDirection());
    resampler->SetDefaultPixelValue(100);

    // Write result
    auto writer = itk::ImageFileWriter<ImageType>::New();
    std::string aux = "/precious_phantom/outputRegisteredImage.mha";
    std::string output_image_path = CURAN_COPIED_RESOURCE_PATH + aux;
    writer->SetFileName(output_image_path);
    writer->SetInput(resampler->GetOutput());

    try {
        writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }


/*
    // Apply transformation
    auto resampler = itk::ResampleImageFilter<ImageType, ImageType>::New();
    resampler->SetTransform(registration->GetTransform());
    resampler->SetInput(movingImageReader->GetOutput());
    resampler->SetSize(fixedImageReader->GetOutput()->GetLargestPossibleRegion().GetSize());
    resampler->SetOutputOrigin(fixedImageReader->GetOutput()->GetOrigin());
    resampler->SetOutputSpacing(fixedImageReader->GetOutput()->GetSpacing());
    resampler->SetOutputDirection(fixedImageReader->GetOutput()->GetDirection());
    resampler->SetDefaultPixelValue(100);

    // Write result
    auto writer = itk::ImageFileWriter<ImageType>::New();
    std::string aux = "/precious_phantom/outputRegisteredImage.mha";
    std::string output_image_path = CURAN_COPIED_RESOURCE_PATH + aux;
    writer->SetFileName(output_image_path);
    writer->SetInput(resampler->GetOutput());
    writer->Update();

    // Store parameters and results
    std::ofstream out("registrationDetails.txt");
    out << "Optimizer State: " << optimizer->GetCurrentPosition() << std::endl;
 //   out << "Last Step Length: " << optimizer->GetCachedDerivative() << std::endl;
    out.close();
*/
    std::cout << "madeeeeeee" << std::endl;
    return 0;
}

int main(int argc, char *argv[]) {
        if (argc!=3)
    {
        std::cerr << "Usage: " << argv[0] << " fixedImageFile movingImageFile" << std::endl;
        std::cerr << "The files must be in the post build resource path, in the precious phantom folder." << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string aux = "/precious_phantom/";
    std::string fixed_image{argv[1]};
    std::string fixed_image_path = CURAN_COPIED_RESOURCE_PATH + aux + fixed_image;
    std::string moving_image{argv[2]};
    std::string moving_image_path = CURAN_COPIED_RESOURCE_PATH + aux + moving_image;
 
    RegisterImages(fixed_image_path, moving_image_path);
    return 0;
}



#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
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
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"


class CommandIterationUpdate : public itk::Command
{
public:
  using Self = CommandIterationUpdate;
  using Superclass = itk::Command;
  using Pointer = itk::SmartPointer<Self>;
  itkNewMacro(Self);
 
protected:
  CommandIterationUpdate() = default;
 
public:
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using OptimizerPointer = const OptimizerType *;
  void
  Execute(itk::Object * caller, const itk::EventObject & event) override
  {
    Execute((const itk::Object *)caller, event);
  }
  void
  Execute(const itk::Object * object, const itk::EventObject & event) override
  {
    auto optimizer = static_cast<OptimizerPointer>(object);
    if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    }
    std::cout << optimizer->GetCurrentIteration() << "   ";
    std::cout << optimizer->GetValue() << "   ";
    std::cout << optimizer->GetCurrentPosition() << std::endl;
  }
};


template <typename TRegistration>
class RegistrationInterfaceCommand : public itk::Command
{
public:
  using Self = RegistrationInterfaceCommand;
  using Superclass = itk::Command;
  using Pointer = itk::SmartPointer<Self>;
  itkNewMacro(Self);
 
protected:
  RegistrationInterfaceCommand() = default;

public:
  using RegistrationType = TRegistration;
  using RegistrationPointer = RegistrationType *;
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using OptimizerPointer = OptimizerType *;

  void
  Execute(itk::Object * object, const itk::EventObject & event) override
  {
    if (!(itk::MultiResolutionIterationEvent().CheckEvent(&event)))
    {
      return;
    }
    auto registration = static_cast<RegistrationPointer>(object);
    auto optimizer =
      static_cast<OptimizerPointer>(registration->GetModifiableOptimizer());
 
    unsigned int currentLevel = registration->GetCurrentLevel();
    typename RegistrationType::ShrinkFactorsPerDimensionContainerType
      shrinkFactors =
        registration->GetShrinkFactorsPerDimension(currentLevel);
    typename RegistrationType::SmoothingSigmasArrayType smoothingSigmas =
      registration->GetSmoothingSigmasPerLevel();
 
    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Current level = " << currentLevel << std::endl;
    std::cout << "    shrink factor = " << shrinkFactors << std::endl;
    std::cout << "    smoothing sigma = ";
    std::cout << smoothingSigmas[currentLevel] << std::endl;
    std::cout << std::endl;
    if (registration->GetCurrentLevel() == 0)
    {
      optimizer->SetNumberOfIterations(1000);
      optimizer->SetLearningRate(20.00);
      optimizer->SetMinimumStepLength(0.001);
    }
    else
    {
      optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
      optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() *
                                      0.2);
                                        optimizer->SetNumberOfIterations(50);
                                        optimizer->SetLearningRate(10);

    }
  }
  void
  Execute(const itk::Object *, const itk::EventObject &) override
  {
    return;
  }
};

int RegisterImages(const std::string& fixedImagePath, const std::string& movingImagePath){

  constexpr unsigned int Dimension = 3;
  using PixelType = float;
  using FixedImageType = itk::Image<PixelType, Dimension>;
  using MovingImageType = itk::Image<PixelType, Dimension>;
  using TransformType = itk::VersorRigid3DTransform<double>;
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using MetricType = itk::MattesMutualInformationImageToImageMetricv4<FixedImageType, MovingImageType>;
  using InterpolatorType = itk::LinearInterpolateImageFunction<FixedImageType, double>;
  using RegistrationType = itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType, TransformType>;
  using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
  using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
  using TransformInitializerType = itk::CenteredTransformInitializer<TransformType,FixedImageType,MovingImageType>;
  using VersorType = TransformType::VersorType;
  using VectorType = VersorType::VectorType;
  using OptimizerScalesType = OptimizerType::ScalesType;


  auto metric = MetricType::New();
  auto optimizer = OptimizerType::New();
  auto registration = RegistrationType::New();

    auto fixedInterpolator = InterpolatorType::New();
    auto movingInterpolator = InterpolatorType::New();

    unsigned int numberOfBins = 50;
    metric->SetNumberOfHistogramBins(numberOfBins);
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    metric->SetFixedInterpolator(fixedInterpolator);
    metric->SetMovingInterpolator(movingInterpolator);
    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
    RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    double samplingPercentage = 0.60;


 
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
      registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);
    registration->MetricSamplingReinitializeSeed(121213);

  auto initialTransform = TransformType::New();
  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();
 
  fixedImageReader->SetFileName(fixedImagePath);
  movingImageReader->SetFileName(movingImagePath);
 
  registration->SetFixedImage(fixedImageReader->GetOutput());
  registration->SetMovingImage(movingImageReader->GetOutput());


  auto initializer = TransformInitializerType::New();
  initializer->SetTransform(initialTransform);
  initializer->SetFixedImage(fixedImageReader->GetOutput());
  initializer->SetMovingImage(movingImageReader->GetOutput());
  initializer->MomentsOn();
  initializer->InitializeTransform();



  VersorType rotation;
  VectorType axis;
  axis[0] = 0.0;
  axis[1] = 0.0;
  axis[2] = 1.0;
  constexpr double angle = 0;
  rotation.Set(axis, angle);
  initialTransform->SetRotation(rotation);

  registration->SetInitialTransform(initialTransform);

 

  OptimizerScalesType optimizerScales(
    initialTransform->GetNumberOfParameters());
  const double translationScale = 1.0 / 1000.0;
  optimizerScales[0] = 100.0;
  optimizerScales[1] = 100.0;
  optimizerScales[2] = 100.0;
  optimizerScales[3] = translationScale;
  optimizerScales[4] = translationScale;
  optimizerScales[5] = translationScale;
  optimizer->SetScales(optimizerScales);
  
  optimizer->SetMinimumStepLength(0.001);

  optimizer->SetRelaxationFactor(0.5);

 
  optimizer->SetReturnBestParametersAndValue(true);
  auto observer = CommandIterationUpdate::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  using CommandType = RegistrationInterfaceCommand<RegistrationType>;
  CommandType::Pointer command = CommandType::New();
  registration->AddObserver(itk::MultiResolutionIterationEvent(), command);

  constexpr unsigned int numberOfLevels = 3;
 RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
 shrinkFactorsPerLevel.SetSize(3);
 shrinkFactorsPerLevel[0] = 4;
 shrinkFactorsPerLevel[1] = 2;
 shrinkFactorsPerLevel[2] = 1;
 RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
 smoothingSigmasPerLevel.SetSize(3);
 smoothingSigmasPerLevel[0] = 4;
 smoothingSigmasPerLevel[1] = 0;
 smoothingSigmasPerLevel[2] = 0;
 registration->SetNumberOfLevels(numberOfLevels);
 registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);
 registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);

 
  try
  {
    registration->Update();
    std::cout << "Optimizer stop condition: "
              << registration->GetOptimizer()->GetStopConditionDescription()
              << std::endl;
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cerr << "ExceptionObject caught !" << std::endl;
    std::cerr << err << std::endl;
    return EXIT_FAILURE;
  }
 
  const TransformType::ParametersType finalParameters =
    registration->GetOutput()->Get()->GetParameters();
 
  const double       versorX = finalParameters[0];
  const double       versorY = finalParameters[1];
  const double       versorZ = finalParameters[2];
  const double       finalTranslationX = finalParameters[3];
  const double       finalTranslationY = finalParameters[4];
  const double       finalTranslationZ = finalParameters[5];
  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();
  const double       bestValue = optimizer->GetValue();
 

  std::cout << std::endl << std::endl;
  std::cout << "Result = " << std::endl;
  std::cout << " versor X      = " << versorX << std::endl;
  std::cout << " versor Y      = " << versorY << std::endl;
  std::cout << " versor Z      = " << versorZ << std::endl;
  std::cout << " Translation X = " << finalTranslationX << std::endl;
  std::cout << " Translation Y = " << finalTranslationY << std::endl;
  std::cout << " Translation Z = " << finalTranslationZ << std::endl;
  std::cout << " Iterations    = " << numberOfIterations << std::endl;
  std::cout << " Metric value  = " << bestValue << std::endl;
 

  auto finalTransform = TransformType::New();
 
  finalTransform->SetFixedParameters(
    registration->GetOutput()->Get()->GetFixedParameters());
  finalTransform->SetParameters(finalParameters);
 
  // Software Guide : BeginCodeSnippet
  TransformType::MatrixType matrix = finalTransform->GetMatrix();
  TransformType::OffsetType offset = finalTransform->GetOffset();
  std::cout << "Matrix = " << std::endl << matrix << std::endl;
  std::cout << "Offset = " << std::endl << offset << std::endl;

  using ResampleFilterType =
    itk::ResampleImageFilter<MovingImageType, FixedImageType>;
 
  auto resampler = ResampleFilterType::New();
 
  resampler->SetTransform(finalTransform);
  resampler->SetInput(movingImageReader->GetOutput());
 
  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();
 
  resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
  resampler->SetOutputOrigin(fixedImage->GetOrigin());
  resampler->SetOutputSpacing(fixedImage->GetSpacing());
  resampler->SetOutputDirection(fixedImage->GetDirection());
  resampler->SetDefaultPixelValue(100);
 
 // Write result
    auto writer = itk::ImageFileWriter<MovingImageType>::New();
    std::string aux = "/precious_phantom/outputRegisteredImage.mha";
    std::string output_image_path = CURAN_COPIED_RESOURCE_PATH + aux;
    writer->SetFileName(output_image_path);
    writer->SetInput(resampler->GetOutput());

    try {
        writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }
  return 0;
}


int main(int argc, char *argv[]) {
        if (argc!=3)
    {
        std::cerr << "Usage: " << argv[0] << " fixedImageFile movingImageFile" << std::endl;
        std::cerr << "The files must be in the post build resource path, in the precious phantom folder." << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string aux = "/precious_phantom/";
    std::string fixed_image{argv[1]};
    std::string fixed_image_path = CURAN_COPIED_RESOURCE_PATH + aux + fixed_image;
          std::string moving_image{argv[2]};
    std::string moving_image_path = CURAN_COPIED_RESOURCE_PATH + aux + moving_image;
 
    RegisterImages(fixed_image_path, moving_image_path);
    return 0;
}