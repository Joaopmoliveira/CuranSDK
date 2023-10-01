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
#include <optional>
#include <nlohmann/json.hpp>

const double pi = std::atan(1) * 4;

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;
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

  try{
    rescale->Update();
  } catch (const itk::ExceptionObject &e){
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
  itk::SmartPointer<RegistrationType> registration;

  void set_registration(itk::SmartPointer<RegistrationType> in_registration)
  {
    registration = in_registration;
  }

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
    if (itk::IterationEvent().CheckEvent(&event))
    {
      auto pos = optimizer->GetCurrentPosition();

      const TransformType::ParametersType finalParameters = registration->GetOutput()->Get()->GetParameters();
      std::cout << "parameters: " << finalParameters << "\n";
      const TransformType::ParametersType finalFixedParameters = registration->GetOutput()->Get()->GetFixedParameters();
      std::cout << "fixed parameters: " << finalFixedParameters << "\n";
      auto finalTransform = TransformType::New();

      finalTransform->SetFixedParameters(finalFixedParameters);
      finalTransform->SetParameters(pos);

      TransformType::MatrixType matrix = finalTransform->GetMatrix();
      TransformType::OffsetType offset = finalTransform->GetOffset();
      auto current_transform = vsg::translate(0.0, 0.0, 0.0);

      std::cout << "Transformation Matrix: \n" << matrix << std::endl;
      std::cout << "Transformation Offset: \n" << offset << std::endl;

      current_transform(3, 0) = finalParameters[3] / 1000.0;
      current_transform(3, 1) = finalParameters[4] / 1000.0;
      current_transform(3, 2) = finalParameters[5] / 1000.0;

      for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
          current_transform(row, col) = matrix(row, col);

      if (moving_pointer_to_volume != nullptr)
        moving_pointer_to_volume->update_transform(current_transform);
        //moving_pointer_to_volume->update_transform(vsg::translate(-pos[3]/1000,-pos[4]/1000,-pos[5]/1000)*vsg::rotate(vsg::radians(-pos[0]),1.0,0.0,0.0)*vsg::rotate(vsg::radians(-pos[1]),0.0,1.0,0.0)*vsg::rotate(vsg::radians(-pos[2]),0.0,0.0,1.0));
        //std::cout << "pos: " << pos << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }
  }
};

int main(int argc, char **argv)
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

  auto initialTransform = TransformType::New();

  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();

  //std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_ct.mha"};
  //std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/ct_fixed.mha"};
  //std::string dirName{"C:/Users/SURGROB7/reconstruction_results.mha"};
  std::string dirName{"C:/Users/SURGROB7/Desktop/Manuel_Carvalho/precious_phantom1.mha"};
  fixedImageReader->SetFileName(dirName);

  //std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/training_001_mr_T1.mha"};
  //std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/mri_move.mha"};
  //std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/mri_move_transf.mha"};
  //std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha"};
  std::string dirName2{"C:/Users/SURGROB7/Desktop/Manuel_Carvalho/precious_phantom_transformed.mha"};
  movingImageReader->SetFileName(dirName2);

  try{
    fixedImageReader->Update();
    movingImageReader->Update();
  } catch (...) {
    return 1;
  }

  ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
  ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

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

  initializer->MomentsOn();

  initializer->InitializeTransform();

  using VersorType = TransformType::VersorType;
  using VectorType = VersorType::VectorType;
  VersorType rotation;
  VectorType axis;
  axis[0] = 0.0;
  axis[1] = 1.0;
  axis[2] = 0.0;
  constexpr double angle = 3.141592;
  rotation.Set(axis, angle);
  VectorType translation;
  translation[0] = 0.0;
  translation[1] = 0.0;
  translation[2] = 0.0;

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

  auto direction = pointer2fixedimage->GetDirection();
  auto origin = pointer2fixedimage->GetOrigin();
  auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
  fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
  fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
  fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

  for (size_t row = 0; row < 3; ++row)
    for (size_t col = 0; col < 3; ++col)
      fixed_homogenenous_transformation(col, row) = direction(row, col);

  volume_fixed->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

  auto casted_volume_fixed = volume_fixed->cast<curan::renderable::Volume>();
  auto updater = [pointer2fixedimage](vsg::floatArray3D &image){ updateBaseTexture3D(image, pointer2fixedimage); };
  casted_volume_fixed->update_volume(updater);

  ImageType::RegionType region_moving = pointer2movingimage->GetLargestPossibleRegion();
  ImageType::SizeType size_itk_moving = region_moving.GetSize();
  ImageType::SpacingType spacing_moving = pointer2movingimage->GetSpacing();

  volumeinfo.width = size_itk_moving.GetSize()[0];
  volumeinfo.height = size_itk_moving.GetSize()[1];
  volumeinfo.depth = size_itk_moving.GetSize()[2];
  volumeinfo.spacing_x = spacing_moving[0];
  volumeinfo.spacing_y = spacing_moving[1];
  volumeinfo.spacing_z = spacing_moving[2];

  auto volume_moving = curan::renderable::Volume::make(volumeinfo);
  window << volume_moving;

  auto casted_volume_moving = volume_moving->cast<curan::renderable::Volume>();
  auto updater_moving = [pointer2movingimage](vsg::floatArray3D &image) { updateBaseTexture3D(image, pointer2movingimage); };

  casted_volume_moving->update_volume(updater_moving);

  auto observer = CommandType::New();
  observer->set_pointer(casted_volume_moving);
  observer->set_registration(registration);
  optimizer->AddObserver(itk::StartEvent(), observer);
  optimizer->AddObserver(itk::IterationEvent(), observer);
  optimizer->AddObserver(itk::EndEvent(), observer);

  std::thread mover_thread{[&](){
    registration->Update();
      const TransformType::ParametersType finalParameters = registration->GetOutput()->Get()->GetParameters();
  auto finalTransform = TransformType::New();

  finalTransform->SetFixedParameters(registration->GetOutput()->Get()->GetFixedParameters());
  finalTransform->SetParameters(finalParameters);

  TransformType::MatrixType matrix = finalTransform->GetMatrix();
  TransformType::OffsetType offset = finalTransform->GetOffset();

  std::cout << "Transformation Matrix: \n" << matrix << std::endl;
  std::cout << "Transformation Offset: \n" << offset << std::endl;

  std::stringstream matrix_value;
  for (size_t y = 0; y < 3; ++y)
  {
    for (size_t x = 0; x < 3; ++x)
    {
      float matrix_entry = matrix[x][y];
      matrix_value << matrix_entry << " ";
    }
    matrix_value << "\n ";
  }
  auto current_transform = vsg::translate(0.0, 0.0, 0.0);

  current_transform(3, 0) = finalParameters[3] / 1000.0;
  current_transform(3, 1) = finalParameters[4] / 1000.0;
  current_transform(3, 2) = finalParameters[5] / 1000.0;

  for (size_t row = 0; row < 3; ++row)
    for (size_t col = 0; col < 3; ++col)
      current_transform(row, col) = matrix(row, col);

  volume_moving->cast<curan::renderable::Volume>()->update_transform(current_transform);

  nlohmann::json registration_transformation;
  registration_transformation["Matrix"] = matrix_value.str();
  registration_transformation["Offset"] = offset;

  std::ofstream output_file{"C:/Users/SURGROB7/registration_results.json"};
  output_file << registration_transformation;

  using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
  auto resampler = ResampleFilterType::New();

  ImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  resampler->SetTransform(finalTransform);
  resampler->SetInput(movingImageReader->GetOutput());
  resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
  resampler->SetOutputOrigin(fixedImage->GetOrigin());
  resampler->SetOutputSpacing(fixedImage->GetSpacing());
  resampler->SetOutputDirection(fixedImage->GetDirection());
  resampler->SetDefaultPixelValue(1);

  auto writer = WriterType::New();
  auto caster = CastFilterType::New();
  auto rescaleFilter = RescaleFilterType::New();

  std::string Output1{"Registration_output_image.mha"};
  writer->SetFileName(Output1);

  caster->SetInput(resampler->GetOutput());
  rescaleFilter->SetInput(resampler->GetOutput());
  writer->SetInput(rescaleFilter->GetOutput());

  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);

  writer->Update();
    }
  };

  window.run();
  mover_thread.join();
  return 0;
}