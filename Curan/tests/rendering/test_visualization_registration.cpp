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

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;

using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;

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
    if (itk::StartEvent().CheckEvent(&event))
    {
      std::cout << "Iteration     Value          Position" << std::endl;
    } else if (itk::IterationEvent().CheckEvent(&event))
    {
      std::cout << optimizer->GetCurrentIteration() << "   ";
      std::cout << optimizer->GetValue() << "   ";
      std::cout << optimizer->GetCurrentPosition() << std::endl;

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
    std::printf("spacing x(%f) y(%f) z(%f)\n",volumeinfo.spacing_x,volumeinfo.spacing_y,volumeinfo.spacing_z);
    std::printf("size x(%f) y(%f) z(%f)\n",(double)volumeinfo.width,(double)volumeinfo.height,(double)volumeinfo.depth);
    auto volume_fixed = curan::renderable::Volume::make(volumeinfo);
    window << volume_fixed;

    auto casted_volume_fixed = volume_fixed->cast<curan::renderable::Volume>();
    auto updater = [pointer2fixedimage](vsg::floatArray3D& image){
        updateBaseTexture3D(image, pointer2fixedimage);
    };
    casted_volume_fixed->update_texture(updater);

    ImageType::RegionType region_moving = pointer2movingimage->GetLargestPossibleRegion();
    ImageType::SizeType size_itk_moving = region_moving.GetSize();
    ImageType::SpacingType spacing_moving = pointer2movingimage->GetSpacing();

    volumeinfo.width = size_itk_moving.GetSize()[0]; 
    volumeinfo.height = size_itk_moving.GetSize()[1];
    volumeinfo.depth = size_itk_moving.GetSize()[2];
    volumeinfo.spacing_x = spacing_moving[0];
    volumeinfo.spacing_y = spacing_moving[1];
    volumeinfo.spacing_z = spacing_moving[2];
    std::printf("spacing x(%f) y(%f) z(%f)\n",volumeinfo.spacing_x,volumeinfo.spacing_y,volumeinfo.spacing_z);
    std::printf("size x(%f) y(%f) z(%f)\n",(double)volumeinfo.width,(double)volumeinfo.height,(double)volumeinfo.depth);
    auto volume_moving = curan::renderable::Volume::make(volumeinfo);
    window << volume_moving;

    auto casted_volume_moving = volume_moving->cast<curan::renderable::Volume>();
    auto updater_moving = [pointer2movingimage](vsg::floatArray3D& image){
        updateBaseTexture3D(image, pointer2movingimage);
    };
    casted_volume_moving->update_texture(updater_moving);
    casted_volume_moving->update_transform(vsg::translate(0.3,0.0,0.0));
    window.run();
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
// clean up done automatically thanks to ref_ptr<>
return 0;
}