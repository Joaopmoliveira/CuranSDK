#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkEuler3DTransform.h"
#include "itkScaleTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkVersorRigid3DTransform.h"
#include "itkVersorRigid3DTransformOptimizer.h"
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

#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"

double pi = std::atan(1)*4;

using PixelType = float;
using PixelType_char = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImageType_char = itk::Image<PixelType_char, 2>;
using TransformType = itk::VersorRigid3DTransform<double>;

using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
//using OptimizerType = itk::VersorRigid3DTransformOptimizer;
using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;

using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<OutputPixelType, Dimension>;
using OutputSliceType = itk::Image<OutputPixelType, 2>;

void function(curan::ui::ImageDisplay* image_display, itk::ExtractImageFilter<OutputImageType, OutputSliceType>* resampler){
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ImageType_char::Pointer pointer_to_block_of_memory = resampler->GetOutput();
    auto lam = [pointer_to_block_of_memory](SkPixmap& requested) {
        ImageType_char::RegionType region = pointer_to_block_of_memory->GetLargestPossibleRegion();
        ImageType_char::SizeType size_itk = region.GetSize();
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
	    SkPixmap map{inf,pointer_to_block_of_memory->GetBufferPointer(),row_size};
	    requested = map;
	    return;
    };
    image_display->update_image(lam);
}

void updateBaseTexture3D(vsg::floatArray3D& image, ImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try{
        rescale->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    ImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<ImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        ImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

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

    /* if (!(itk::MultiResolutionIterationEvent().CheckEvent(&event)))
    {
      return;
    } */

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
    std::cout << " Current level = " << currentLevel +1 << std::endl;
    /* std::cout << "    shrink factor = " << shrinkFactors << std::endl;
    std::cout << "    smoothing sigma = ";
    std::cout << smoothingSigmas[currentLevel] << std::endl; */
    std::cout << std::endl;

    /* if (registration->GetCurrentLevel() == 0)
    {
      optimizer->SetLearningRate(16.00);
      optimizer->SetMinimumStepLength(2.5);
    }
    else
    {
      optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
      optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() * 0.2);
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
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using OptimizerPointer = const OptimizerType *;
  curan::renderable::Volume* moving_pointer_to_volume = nullptr;

  void set_pointer(curan::renderable::Volume* in_moving_pointer_to_volume){
    moving_pointer_to_volume = in_moving_pointer_to_volume;
  }

  void Execute(itk::Object * caller, const itk::EventObject & event) override
  {
    Execute((const itk::Object *)caller, event);
  }
  void Execute(const itk::Object * object, const itk::EventObject & event) override
  {
    auto optimizer = static_cast<OptimizerPointer>(object);
    if (itk::StartEvent().CheckEvent(&event))
    {
      std::cout << "Iter     Value          Position" << std::endl;
    } else if (itk::IterationEvent().CheckEvent(&event))
    {
      auto pos = optimizer->GetCurrentPosition();
      if(moving_pointer_to_volume!=nullptr){
        moving_pointer_to_volume->update_transform(vsg::translate(-pos[3]/1000,-pos[4]/1000,-pos[5]/1000)*vsg::rotate(vsg::radians(-pos[0]),1.0,0.0,0.0)*vsg::rotate(vsg::radians(-pos[1]),0.0,1.0,0.0)*vsg::rotate(vsg::radians(-pos[2]),0.0,0.0,1.0));
      }
  
      /* std::cout << optimizer->GetCurrentIteration() << "   ";
      std::cout << optimizer->GetValue() << "   ";
      std::cout << optimizer->GetCurrentPosition() << std::endl; */
      std::this_thread::sleep_for(std::chrono::milliseconds(7));

    } else if (itk::MultiResolutionIterationEvent().CheckEvent(&event))
    {
      std::cout << "aaaa" << std::endl;
    } else if (itk::EndEvent().CheckEvent(&event))
    {
      std::cout << "Finish" << std::endl;
      std::cout << std::endl << std::endl;
    }
  }
};

int main(int argc, char** argv) {
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

  using FixedImageReaderType = itk::ImageFileReader<ImageType>;
  using MovingImageReaderType = itk::ImageFileReader<ImageType>;
  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();


  std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_ct.mha"};
  fixedImageReader->SetFileName(dirName);

  std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_mr_T1.mha"};
  movingImageReader->SetFileName(dirName2);

  try{
    fixedImageReader->Update();
    movingImageReader->Update();
  } catch (...) { 
    return 1;
  }

ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();
//ImageType::PointType newOrigin;
//newOrigin.Fill(1.0);
//pointer2movingimage->SetOrigin(newOrigin);

//std::printf("fixed position x(%f) y(%f) z(%f)\n",pointer2fixedimage->GetOrigin()[0],pointer2fixedimage->GetOrigin()[1],pointer2fixedimage->GetOrigin()[2]);
//std::printf("moving position x(%f) y(%f) z(%f)\n",pointer2movingimage->GetOrigin()[0],pointer2movingimage->GetOrigin()[1],pointer2movingimage->GetOrigin()[2]);
registration->SetFixedImage(pointer2fixedimage);
registration->SetMovingImage(pointer2movingimage);

using TransformInitializerType =
itk::CenteredTransformInitializer<TransformType,
                                    ImageType,
                                    ImageType>;
auto initializer = TransformInitializerType::New();

initializer->SetTransform(initialTransform);
initializer->SetFixedImage(fixedImageReader->GetOutput());
initializer->SetMovingImage(movingImageReader->GetOutput());


//initializer->MomentsOn();
initializer->GeometryOn();

initializer->InitializeTransform();

using VersorType = TransformType::VersorType;
using VectorType = VersorType::VectorType;
VersorType rotation;
VectorType axis;
axis[0] = 0.3165373;
axis[1] =  -0.0143763;
axis[2] = -0.9484711;
constexpr double angle = 0.0875013;
rotation.Set(axis, angle);
VectorType translation;
translation[0] =17.7001;
translation[1] = -33.3714;
translation[2] = -21.292;
VectorType offset1;
offset1[0] = 4.57104 + 40;
offset1[1] = -17.3421 + 40;
offset1[2] = -25.9166;
initialTransform->SetRotation(rotation);
initialTransform->SetTranslation(translation);
initialTransform->SetOffset(offset1);
initialTransform_2->SetRotation(rotation);
initialTransform_2->SetTranslation(translation);
initialTransform_2->SetOffset(offset1);
std::cout << "initial transform matrix: " << initialTransform->GetMatrix() << std::endl;
std::cout << "initial transform offset: " << initialTransform->GetOffset() << std::endl << std::endl;
std::cout << "initial transform 2 offset: " << initialTransform_2->GetOffset() << std::endl << std::endl;

std::cout << "initial translaction: " << initialTransform->GetTranslation() << std::endl;
std::cout << "initial rotation: " << initialTransform->GetVersor() << std::endl;

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

try{
  ImageType::RegionType region = pointer2fixedimage->GetLargestPossibleRegion();
  ImageType::SizeType size_itk = region.GetSize();
  ImageType::SpacingType spacing = pointer2fixedimage->GetSpacing();
  
  curan::renderable::Window::Info info;
  info.api_dump = false;
  info.display = "";
  info.full_screen = false;
  info.is_debug = false;
  info.screen_number = 0;
  info.title = "myviewer";
  curan::renderable::Window::WindowSize size{1000, 800};
  info.window_size = size;
  curan::renderable::Window window{info};

  curan::renderable::Volume::Info volumeinfo;
  volumeinfo.width = size_itk.GetSize()[0]; 
  volumeinfo.height = size_itk.GetSize()[1];
  volumeinfo.depth = size_itk.GetSize()[2];
  volumeinfo.spacing_x = spacing[0];
  volumeinfo.spacing_y = spacing[1];
  volumeinfo.spacing_z = spacing[2];



  auto volume_fixed = curan::renderable::Volume::make(volumeinfo);
  window << volume_fixed;

  auto casted_volume_fixed = volume_fixed->cast<curan::renderable::Volume>();
  auto updater = [pointer2fixedimage](vsg::floatArray3D& image){
      updateBaseTexture3D(image, pointer2fixedimage);
  };
  casted_volume_fixed->update_volume(updater);

  //std::printf("fixed image dimension x(%i) y(%i) z(%i)\n",volumeinfo.width,volumeinfo.height,volumeinfo.depth);

  ImageType::RegionType region_moving = pointer2movingimage->GetLargestPossibleRegion();
  ImageType::SizeType size_itk_moving = region_moving.GetSize();
  ImageType::SpacingType spacing_moving = pointer2movingimage->GetSpacing();

  volumeinfo.width = size_itk_moving.GetSize()[0]; 
  volumeinfo.height = size_itk_moving.GetSize()[1];
  volumeinfo.depth = size_itk_moving.GetSize()[2];
  volumeinfo.spacing_x = spacing_moving[0];
  volumeinfo.spacing_y = spacing_moving[1];
  volumeinfo.spacing_z = spacing_moving[2];

  //std::printf("moving image dimension x(%i) y(%i) z(%i)\n",volumeinfo.width,volumeinfo.height,volumeinfo.depth);

  

  auto volume_moving = curan::renderable::Volume::make(volumeinfo);
  window << volume_moving;

  auto casted_volume_moving = volume_moving->cast<curan::renderable::Volume>();
  auto updater_moving = [pointer2movingimage](vsg::floatArray3D& image){
      updateBaseTexture3D(image, pointer2movingimage);
  };

  casted_volume_moving->update_volume(updater_moving);
  //casted_volume_moving->update_transform(vsg::translate(0.3,0.0,0.0));

  using CommanddType2 = CommandIterationUpdate;
  auto observer = CommanddType2::New();
  observer->set_pointer(casted_volume_moving);
  optimizer->AddObserver(itk::StartEvent(), observer);
  optimizer->AddObserver(itk::IterationEvent(), observer);
  optimizer->AddObserver(itk::MultiResolutionIterationEvent(), observer);
  optimizer->AddObserver(itk::EndEvent(), observer);

  using CommandType = RegistrationInterfaceCommand<RegistrationType>;
  auto command = CommandType::New();
  registration->AddObserver(itk::MultiResolutionIterationEvent(), command);
    
  auto mover = [&registration](){
      registration->Update();
  };
  std::thread mover_thread{mover};
  window.run();
  mover_thread.join();
  
  window.transverse_identifiers(
          [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                  &map) {
              for (auto &p : map){
                  std::cout << "Object contained: " << p.first << '\n';
              }
  });

} catch (const std::exception& e) {
     std::cerr << "Exception thrown : " << e.what() << std::endl;
    return 1;
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

  std::cout << "final translaction: " << finalTransform->GetTranslation() << std::endl;
  std::cout << "final versor: " << finalTransform->GetVersor() << std::endl;  
 
  using ResampleFilterType =
    itk::ResampleImageFilter<ImageType, ImageType>;

  auto resampler = ResampleFilterType::New();

  resampler->SetTransform(finalTransform);
  resampler->SetInput(movingImageReader->GetOutput());

  auto outputtemporary = movingImageReader->GetOutput();
  using ConstIteratorType = itk::ImageRegionConstIterator<ImageType>;
  using IteratorType = itk::ImageRegionIterator<ImageType>;

  ImageType::Pointer image = movingImageReader->GetOutput();
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

  ImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
  resampler->SetOutputOrigin(fixedImage->GetOrigin());
  resampler->SetOutputSpacing(fixedImage->GetSpacing());
  resampler->SetOutputDirection(fixedImage->GetDirection());
  resampler->SetDefaultPixelValue(0);

  using OutputPixelType = unsigned char;
  using OutputImageType = itk::Image<OutputPixelType, Dimension>;
  using CastFilterType =
    itk::CastImageFilter<ImageType, OutputImageType>;
  using RescaleFilterType =
    itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
  using WriterType = itk::ImageFileWriter<OutputImageType>;

  auto writer = WriterType::New();
  auto caster = CastFilterType::New();
  auto rescaleFilter = RescaleFilterType::New();

  std::string Output1{"outputImagefile.mha"};
  writer->SetFileName(Output1);

  caster->SetInput(resampler->GetOutput());
  rescaleFilter->SetInput(resampler->GetOutput());
  writer->SetInput(rescaleFilter->GetOutput());

  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  
  writer->Update();

  using DifferenceFilterType =
    itk::SubtractImageFilter<ImageType, ImageType, ImageType>;
  auto difference = DifferenceFilterType::New();

  using RescalerType =
    itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
  auto intensityRescaler = RescalerType::New();

  intensityRescaler->SetInput(difference->GetOutput());
  intensityRescaler->SetOutputMinimum(0);
  intensityRescaler->SetOutputMaximum(255);

  difference->SetInput1(fixedImageReader->GetOutput());
  difference->SetInput2(resampler->GetOutput());

  resampler->SetDefaultPixelValue(0);

  auto writer2 = WriterType::New();
  writer2->SetInput(intensityRescaler->GetOutput());

  // Compute the difference image between the
  // fixed and resampled moving image.

  std::string Output2{"differenceAfterRegistration.mha"};
  writer2->SetFileName(Output2);
  writer2->Update();


  using IdentityTransformType = itk::IdentityTransform<double, Dimension>;
  auto identity = IdentityTransformType::New();
  // Compute the difference image between the
  // fixed and moving image before registration.

  resampler->SetTransform(initialTransform_2);
  std::string Output3{"differenceBeforeRegistration.mha"};
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

  ImageType::RegionType inputRegion = fixedImage->GetLargestPossibleRegion();
  ImageType::SizeType  size = inputRegion.GetSize();
  ImageType::IndexType start = inputRegion.GetIndex();
  ImageType::SpacingType spacing = pointer2fixedimage->GetSpacing();

  std::cout << "Image spacing: " << spacing << std::endl;

  // Select one slice as output
  size[2] = 0;
  start[2] = 20;
  /* size[1] = 0;
  start[1] = 256; */
  ImageType::RegionType desiredRegion;
  desiredRegion.SetSize(size);
  desiredRegion.SetIndex(start);

  extractor->SetExtractionRegion(desiredRegion);
  using SliceWriterType = itk::ImageFileWriter<OutputSliceType>;
  auto sliceWriter = SliceWriterType::New();
  sliceWriter->SetInput(extractor->GetOutput());


  std::cout << "initial transform matrix: " << initialTransform_2->GetMatrix() << std::endl;
  std::cout << "initial transform offset: " << initialTransform_2->GetOffset() << std::endl << std::endl;

  std::cout << "initial translaction: " << initialTransform_2->GetTranslation() << std::endl;
  std::cout << "initial rotation: " << initialTransform_2->GetVersor() << std::endl;

  extractor->SetInput(rescaleFilter->GetOutput());
  resampler->SetTransform(initialTransform_2);
  std::string Output4{"sliceBeforeRegistration.png"};
  sliceWriter->SetFileName(Output4);
  sliceWriter->Update();

  extractor->SetInput(intensityRescaler->GetOutput());
  resampler->SetTransform(initialTransform_2);
  std::string Output5{"sliceDifferenceBeforeRegistration.png"};
  sliceWriter->SetFileName(Output5);
  sliceWriter->Update();

  resampler->SetTransform(finalTransform);
  std::string Output6{"sliceDifferenceAfterRegistration.png"};
  sliceWriter->SetFileName(Output6);
  sliceWriter->Update();

  extractor->SetInput(rescaleFilter->GetOutput());
  resampler->SetTransform(finalTransform);
  std::string Output7{"sliceAfterRegistration.png"};
  sliceWriter->SetFileName(Output7);
  sliceWriter->Update();


  extractor->SetInput(intensityRescaler->GetOutput());
  resampler->SetTransform(finalTransform);
  try {
    using namespace curan::ui;
    std::unique_ptr<Context> context = std::make_unique<Context>();;
    DisplayParams param{ std::move(context),600,600 };
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    std::unique_ptr<ImageDisplay> image_display = ImageDisplay::make();
    image_display->set_size(SkRect::MakeWH(600,600));
    ImageDisplay* pointer_to = image_display.get();
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display);
    curan::ui::Page page{std::move(container),SK_ColorBLACK};

    auto call = [&](){
        function(pointer_to, extractor);
    };

    std::thread image_generator(call);

    auto rec = viewer->get_size();
    page.propagate_size_change(rec);
    int width = rec.width();
    int height = rec.height();

    ConfigDraw config{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
        auto start = std::chrono::high_resolution_clock::now();
        SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
        auto temp_height = pointer_to_surface->height();
        auto temp_width = pointer_to_surface->width();
        SkCanvas* canvas = pointer_to_surface->getCanvas();
        if (temp_height != height || temp_width != width) {
            rec = SkRect::MakeWH(temp_width, temp_height);
            page.propagate_size_change(rec);
        }
        page.draw(canvas);
        auto signals = viewer->process_pending_signals();
        if (!signals.empty())
            page.propagate_signal(signals.back(),&config);
        glfwPollEvents();
    
        bool val = viewer->swapBuffers();
        if (!val)
            std::cout << "failed to swap buffers\n";
        auto end = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
    }
    image_generator.join();
    return 0;   
    }
catch (std::exception & e ) {
    std::cout << "Failed: " << e.what() << std::endl;
    return 1;
}
// clean up done automatically thanks to ref_ptr<>
return 0;
}