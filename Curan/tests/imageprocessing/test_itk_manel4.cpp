//    INPUTS: brainweb1e1a10f20.mha
//    INPUTS: brainweb1e1a10f20Rot10Tx15.mha
//    ARGUMENTS: ImageRegistration8Output.mhd
//    ARGUMENTS: ImageRegistration8DifferenceBefore.mhd
//    ARGUMENTS: ImageRegistration8DifferenceAfter.mhd
//    OUTPUTS: {ImageRegistration8Output.png}
//    OUTPUTS: {ImageRegistration8DifferenceBefore.png}
//    OUTPUTS: {ImageRegistration8DifferenceAfter.png}
//    OUTPUTS: {ImageRegistration8RegisteredSlice.png}


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

    /* if (registration->GetCurrentLevel() == 0)
    {
      optimizer->SetLearningRate(16.00);
      optimizer->SetMinimumStepLength(2.5);
    }
    else
    {
      optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
      optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() *
                                      0.2);
    } */
  }

  void Execute(const itk::Object *, const itk::EventObject &) override
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
  CommandIterationUpdate() = default;

public:
  /* using RegistrationType = TRegistration;
  using RegistrationPointer = RegistrationType *; */
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
    //auto registration = static_cast<RegistrationPointer>(object);
    auto optimizer = static_cast<OptimizerPointer>(object);
    /* if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    } */
    if (itk::StartEvent().CheckEvent(&event))
    {
      std::cout << "Iteration     Value          Position" << std::endl;
    } else if (itk::IterationEvent().CheckEvent(&event))
    {
      std::cout << optimizer->GetCurrentIteration() << "   ";
      std::cout << optimizer->GetValue() << "   ";
      std::cout << optimizer->GetCurrentPosition() << std::endl;
      //std::cout << "current level: " << registration->GetCurrentLevel() << std::endl;

    } else if (itk::MultiResolutionIterationEvent().CheckEvent(&event))
    {
      std::cout << "aaaa" << std::endl;
    } else if (itk::EndEvent().CheckEvent(&event))
    {
      std::cout << "Finish" << std::endl;
      std::cout << std::endl << std::endl;
    }
    /* std::cout << optimizer->GetCurrentIteration() << "   ";
    std::cout << optimizer->GetValue() << "   ";
    std::cout << optimizer->GetCurrentPosition() << std::endl; */
  }
};

int
main(int argc, char * argv[])
{
/*   if (argc < 4)
  {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile ";
    std::cerr << " outputImagefile  [differenceBeforeRegistration] ";
    std::cerr << " [differenceAfterRegistration] ";
    std::cerr << " [sliceBeforeRegistration] ";
    std::cerr << " [sliceDifferenceBeforeRegistration] ";
    std::cerr << " [sliceDifferenceAfterRegistration] ";
    std::cerr << " [sliceAfterRegistration] " << std::endl;
    return EXIT_FAILURE;
  } */
constexpr unsigned int Dimension = 3;
using PixelType = float;
using FixedImageType = itk::Image<PixelType, Dimension>;
using MovingImageType = itk::Image<PixelType, Dimension>;

using TransformType = itk::VersorRigid3DTransform<double>;

using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType =
  itk::MattesMutualInformationImageToImageMetricv4<FixedImageType, MovingImageType>;
using RegistrationType = itk::
  ImageRegistrationMethodv4<FixedImageType, MovingImageType, TransformType>;

auto metric = MetricType::New();
auto optimizer = OptimizerType::New();
auto registration = RegistrationType::New();

registration->SetMetric(metric);
registration->SetOptimizer(optimizer);


unsigned int numberOfBins = 50;

metric->SetNumberOfHistogramBins(numberOfBins);

metric->SetUseMovingImageGradientFilter(false);
metric->SetUseFixedImageGradientFilter(false);

auto initialTransform = TransformType::New();
auto initialTransform_2 = TransformType::New();

using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
auto fixedImageReader = FixedImageReaderType::New();
auto movingImageReader = MovingImageReaderType::New();


std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_ct.mha"};
//std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/ct_fixed.mha"};
fixedImageReader->SetFileName(dirName);

std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_mr_T1.mha"};
//std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/mri_move_transf.mha"};
movingImageReader->SetFileName(dirName2);


registration->SetFixedImage(fixedImageReader->GetOutput());
registration->SetMovingImage(movingImageReader->GetOutput());



using TransformInitializerType =
  itk::CenteredTransformInitializer<TransformType,
                                    FixedImageType,
                                    MovingImageType>;
auto initializer = TransformInitializerType::New();

initializer->SetTransform(initialTransform);
initializer->SetFixedImage(fixedImageReader->GetOutput());
initializer->SetMovingImage(movingImageReader->GetOutput());


initializer->MomentsOn();

initializer->InitializeTransform();

using VersorType = TransformType::VersorType;
using VectorType = VersorType::VectorType;
VersorType rotation;
VectorType axis;
axis[0] = 1.0;
axis[1] = 0.0;
axis[2] = 0.0;
constexpr double angle = 1.0;
rotation.Set(axis, angle);
VectorType translation;
translation[0] = 0.0;
translation[1] = 0.0;
translation[2] = 0.0;
initialTransform->SetRotation(rotation);
//initialTransform->SetTranslation(translation);

std::cout << "initial translaction: " << initialTransform->GetTranslation() << std::endl;
std::cout << "initial rotation: " << initialTransform->GetVersor() << std::endl;

initialTransform_2 = initialTransform;
registration->SetInitialTransform(initialTransform);

  using OptimizerScalesType = OptimizerType::ScalesType;
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
  optimizer->SetNumberOfIterations(2000);
  optimizer->SetLearningRate(1);
  optimizer->SetMinimumStepLength(0.001);
  optimizer->SetReturnBestParametersAndValue(true);
  optimizer->SetNumberOfThreads(8);
  itk::SizeValueType value{10};
  optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(0.8);

  // Create the Command observer and register it with the optimizer.
  //
  using CommanddType2 = CommandIterationUpdate;
  auto observer = CommanddType2::New();
  optimizer->AddObserver(itk::StartEvent(), observer);
  optimizer->AddObserver(itk::IterationEvent(), observer);
  optimizer->AddObserver(itk::MultiResolutionIterationEvent(), observer);
  optimizer->AddObserver(itk::EndEvent(), observer);

  using CommandType = RegistrationInterfaceCommand<RegistrationType>;
  auto command = CommandType::New();
  registration->AddObserver(itk::MultiResolutionIterationEvent(), command);

  //
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

  // Print out results
  //
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


  TransformType::MatrixType matrix = finalTransform->GetMatrix();
  TransformType::OffsetType offset = finalTransform->GetOffset();
  std::cout << std::endl << "Matrix = " << std::endl << matrix << std::endl;
  std::cout << "Offset = " << std::endl << offset << std::endl;
 
  using ResampleFilterType =
    itk::ResampleImageFilter<MovingImageType, FixedImageType>;

  auto resampler = ResampleFilterType::New();

  resampler->SetTransform(finalTransform);
  resampler->SetInput(movingImageReader->GetOutput());

  auto outputtemporary = movingImageReader->GetOutput();
  using ConstIteratorType = itk::ImageRegionConstIterator<MovingImageType>;
  using IteratorType = itk::ImageRegionIterator<MovingImageType>;

  MovingImageType::Pointer image = movingImageReader->GetOutput();
  ConstIteratorType constIterator(image,image->GetRequestedRegion());
  IteratorType iterator(image,image->GetRequestedRegion());

  double maximum_value = -10000;
  double minimum_value = 10000;
  for( iterator.GoToBegin(); !iterator.IsAtEnd() ; ++iterator){
    if(iterator.Get()>maximum_value)
      maximum_value = iterator.Get();
    if(iterator.Get() < minimum_value)
      minimum_value = iterator.Get();
  }

  std::cout << std::endl << "minimum value is : " << minimum_value << "\nmaximum value is: " << maximum_value << std::endl; 

  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
  resampler->SetOutputOrigin(fixedImage->GetOrigin());
  resampler->SetOutputSpacing(fixedImage->GetSpacing());
  resampler->SetOutputDirection(fixedImage->GetDirection());
  resampler->SetDefaultPixelValue(1);

  using OutputPixelType = unsigned char;
  using OutputImageType = itk::Image<OutputPixelType, Dimension>;
  using CastFilterType =
    itk::CastImageFilter<FixedImageType, OutputImageType>;
  using RescaleFilterType =
    itk::RescaleIntensityImageFilter<MovingImageType, OutputImageType>;
  using WriterType = itk::ImageFileWriter<OutputImageType>;

  auto writer = WriterType::New();
  auto caster = CastFilterType::New();
  auto rescaleFilter = RescaleFilterType::New();

  std::string Output1{"newmovedimage1.mha"};
  writer->SetFileName(Output1);

  caster->SetInput(resampler->GetOutput());
  rescaleFilter->SetInput(resampler->GetOutput());
  writer->SetInput(rescaleFilter->GetOutput());

  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  
  writer->Update();

  using DifferenceFilterType =
    itk::SubtractImageFilter<FixedImageType, FixedImageType, FixedImageType>;
  auto difference = DifferenceFilterType::New();

  using RescalerType =
    itk::RescaleIntensityImageFilter<FixedImageType, OutputImageType>;
  auto intensityRescaler = RescalerType::New();

  intensityRescaler->SetInput(difference->GetOutput());
  intensityRescaler->SetOutputMinimum(0);
  intensityRescaler->SetOutputMaximum(255);

  difference->SetInput1(fixedImageReader->GetOutput());
  difference->SetInput2(resampler->GetOutput());

  resampler->SetDefaultPixelValue(1);

  auto writer2 = WriterType::New();
  writer2->SetInput(intensityRescaler->GetOutput());

  // Compute the difference image between the
  // fixed and resampled moving image.

  std::string Output2{"newmovedimage2.mha"};
  writer2->SetFileName(Output2);
  writer2->Update();


  using IdentityTransformType = itk::IdentityTransform<double, Dimension>;
  auto identity = IdentityTransformType::New();
  // Compute the difference image between the
  // fixed and moving image before registration.

  resampler->SetTransform(identity);
  std::string Output3{"newmovedimage3.mha"};
  writer2->SetFileName(Output3);
  writer2->Update();

  //
  //  Here we extract slices from the input volume, and the difference volumes
  //  produced before and after the registration.  These slices are presented
  //  as figures in the Software Guide.
  //
  //
  using OutputSliceType = itk::Image<OutputPixelType, 2>;
  using ExtractFilterType =
    itk::ExtractImageFilter<OutputImageType, OutputSliceType>;
  auto extractor = ExtractFilterType::New();
  extractor->SetDirectionCollapseToSubmatrix();
  extractor->InPlaceOn();

  FixedImageType::RegionType inputRegion =
    fixedImage->GetLargestPossibleRegion();
  FixedImageType::SizeType  size = inputRegion.GetSize();
  FixedImageType::IndexType start = inputRegion.GetIndex();

  // Select one slice as output
  /* size[2] = 0;
  start[2] = 90; */
  size[2] = 0;
  start[2] = 20;
  FixedImageType::RegionType desiredRegion;
  desiredRegion.SetSize(size);
  desiredRegion.SetIndex(start);
  extractor->SetExtractionRegion(desiredRegion);
  using SliceWriterType = itk::ImageFileWriter<OutputSliceType>;
  auto sliceWriter = SliceWriterType::New();
  sliceWriter->SetInput(extractor->GetOutput());

  extractor->SetInput(rescaleFilter->GetOutput());
  resampler->SetTransform(identity);
  std::string Output4{"newmovedimage4.png"};
  sliceWriter->SetFileName(Output4);
  sliceWriter->Update();

  extractor->SetInput(intensityRescaler->GetOutput());
  resampler->SetTransform(identity);
  std::string Output5{"newmovedimage5.png"};
  sliceWriter->SetFileName(Output5);
  sliceWriter->Update();

  resampler->SetTransform(finalTransform);
  std::string Output6{"newmovedimage6.png"};
  sliceWriter->SetFileName(Output6);
  sliceWriter->Update();

  extractor->SetInput(rescaleFilter->GetOutput());
  resampler->SetTransform(finalTransform);
  std::string Output7{"newmovedimage7.png"};
  sliceWriter->SetFileName(Output7);
  sliceWriter->Update();

  return EXIT_SUCCESS;
}