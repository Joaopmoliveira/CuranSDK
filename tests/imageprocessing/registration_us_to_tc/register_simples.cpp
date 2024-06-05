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
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkRelabelComponentImageFilter.h"


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
      optimizer->SetNumberOfIterations(500);
      optimizer->SetLearningRate(16.00);
      optimizer->SetMinimumStepLength(0.001);
    }
    else
    {
      optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
      optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() *
                                      0.2);
                                        optimizer->SetNumberOfIterations(200);
                                        optimizer->SetLearningRate(5);

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

    unsigned int numberOfBins = 100;
    metric->SetNumberOfHistogramBins(numberOfBins);
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    metric->SetFixedInterpolator(fixedInterpolator);
    metric->SetMovingInterpolator(movingInterpolator);
    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
    RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    double samplingPercentage = 0.99;


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
  optimizerScales[0] = 1.0;
  optimizerScales[1] = 1.0;
  optimizerScales[2] = 1.0;
  optimizerScales[3] = translationScale;
  optimizerScales[4] = translationScale;
  optimizerScales[5] = translationScale;
  optimizer->SetScales(optimizerScales);
  
  optimizer->SetMinimumStepLength(0.001);

  optimizer->SetRelaxationFactor(0.9);

 
  optimizer->SetReturnBestParametersAndValue(true);
  auto observer = CommandIterationUpdate::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  using CommandType = RegistrationInterfaceCommand<RegistrationType>;
  CommandType::Pointer command = CommandType::New();
  registration->AddObserver(itk::MultiResolutionIterationEvent(), command);

  constexpr unsigned int numberOfLevels = 3;
 RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
 shrinkFactorsPerLevel.SetSize(3);
 shrinkFactorsPerLevel[0] = 1;
 shrinkFactorsPerLevel[1] = 1;
 shrinkFactorsPerLevel[2] = 1;
 RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
 smoothingSigmasPerLevel.SetSize(3);
 smoothingSigmasPerLevel[0] = 2;
 smoothingSigmasPerLevel[1] = 1;
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
  resampler->SetDefaultPixelValue(0);
 
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



