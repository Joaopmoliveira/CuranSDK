#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkNormalVariateGenerator.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
 
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"

int main(int argc, char * argv[]){
  if (argc < 3)
  {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile ";
    std::cerr << "outputImagefile ";
    std::cerr << "[samplingPercentage ] " << std::endl;
    return EXIT_FAILURE;
  }
 
  constexpr unsigned int Dimension = 2;
  using PixelType = float;
 
  using FixedImageType = itk::Image<PixelType, Dimension>;
  using MovingImageType = itk::Image<PixelType, Dimension>;
 
  using TransformType = itk::TranslationTransform<double, Dimension>;
  using OptimizerType = itk::OnePlusOneEvolutionaryOptimizerv4<double>;
  using RegistrationType = itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType>;

  using MetricType =
    itk::MattesMutualInformationImageToImageMetricv4<FixedImageType,
                                                     MovingImageType>;
 
  auto transform = TransformType::New();
  auto optimizer = OptimizerType::New();
  auto metric = MetricType::New();
  auto registration = RegistrationType::New();
 
  registration->SetOptimizer(optimizer);
  registration->SetMetric(metric);
 
  metric->SetNumberOfHistogramBins(20);

  double samplingPercentage = 0.20;
  if (argc > 4)
  {
    samplingPercentage = std::stod(argv[4]);
  }

  registration->SetMetricSamplingPercentage(samplingPercentage);
 
  RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
    RegistrationType::MetricSamplingStrategyEnum::RANDOM;
  registration->SetMetricSamplingStrategy(samplingStrategy);
 
  using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
  using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
 
  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();
 
  fixedImageReader->SetFileName(argv[1]);
  movingImageReader->SetFileName(argv[2]);
 
  registration->SetFixedImage(fixedImageReader->GetOutput());
  registration->SetMovingImage(movingImageReader->GetOutput());
 
  fixedImageReader->Update();
 
 
  using ParametersType = TransformType::ParametersType;
  ParametersType initialParameters(transform->GetNumberOfParameters());
 
  initialParameters[0] = 0.0; 
  initialParameters[1] = 0.0; 
 
  transform->SetParameters(initialParameters);
 
  registration->SetInitialTransform(transform);
  registration->InPlaceOn();

  using GeneratorType = itk::Statistics::NormalVariateGenerator;
 
  auto generator = GeneratorType::New();

  generator->Initialize(12345);
  optimizer->SetNormalVariateGenerator(generator);
  optimizer->Initialize(10);
  optimizer->SetEpsilon(1.0);
  optimizer->SetMaximumIteration(4000);
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
 
  try
  {
    registration->Update();
    std::cout << "Registration completed!" << std::endl;
    std::cout << "Optimizer stop condition: "
              << registration->GetOptimizer()->GetStopConditionDescription()
              << std::endl;
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
  }
 
  ParametersType finalParameters = transform->GetParameters();
 
  double TranslationAlongX = finalParameters[0];
  double TranslationAlongY = finalParameters[1];
 
  unsigned int numberOfIterations = optimizer->GetCurrentIteration();
 
  double bestValue = optimizer->GetValue();
 
  std::cout << "Result = " << std::endl;
  std::cout << " Translation X = " << TranslationAlongX << std::endl;
  std::cout << " Translation Y = " << TranslationAlongY << std::endl;
  std::cout << " Iterations    = " << numberOfIterations << std::endl;
  std::cout << " Metric value  = " << bestValue << std::endl;
 
  return EXIT_SUCCESS;
}