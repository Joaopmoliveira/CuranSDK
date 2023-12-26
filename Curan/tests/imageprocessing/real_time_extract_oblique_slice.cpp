#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
 
#include "itkResampleImageFilter.h"
 
#include "itkEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <optional>


constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;


using InterImageType = itk::Image<InterPixelType, Dimension_out>;

//using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;



using ReaderType = itk::ImageFileReader<InputImageType>;
using WriterType = itk::ImageFileWriter<OutputImageType>;

void updateBaseTexture3D(vsg::floatArray3D &image, InputImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<InputImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<InputImageType, InputImageType>;
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

    InputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<InputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        InputImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}
 

 
int 
main(int argc, char * argv[]) {
 
  auto reader = ReaderType::New();
  auto writer = WriterType::New();
 
  std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
  std::string dirName_output{"extracted_slice.mha"};

  reader->SetFileName(dirName_input);
  writer->SetFileName(dirName_output);

  try
    {
        reader->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        //return;
    }
 

  InputImageType::Pointer pointer_to_block_of_memory = reader->GetOutput();
  InputImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();

  std::cout << "(input) origin : " << pointer_to_block_of_memory->GetOrigin() << std::endl;
  std::cout << "(input) size : " << pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize() << std::endl;
  std::cout << "(input) spacing : " << pointer_to_block_of_memory->GetSpacing() << std::endl;
  std::cout << "(input) direction : " << pointer_to_block_of_memory->GetDirection() << std::endl;

  using FilterType = itk::ResampleImageFilter<InputImageType, OutputImageType>;
  auto filter = FilterType::New();

  //using InterpolatorType = itk::LinearInterpolateImageFunction<InputImageType, double>;
  using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
  auto interpolator = InterpolatorType::New();
  filter->SetInterpolator(interpolator);
  filter->SetDefaultPixelValue(300);

  auto input = pointer_to_block_of_memory;
  auto size = input->GetLargestPossibleRegion().GetSize();
  auto spacing = input->GetSpacing();
  double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);
  double maximum_size = std::max(std::max(size[0], size[1]), size[2]);
  auto new_spacing = spacing;
  new_spacing[0] = minimum_spacing;
  new_spacing[1] = minimum_spacing;
  new_spacing[2] = 0.00001;

  auto out_size = size;
  out_size[0] = maximum_size;
  out_size[1] = maximum_size;
  out_size[2] = maximum_size;

  out_size[2] = 1;
  
  
  
  filter->SetInput(input);

  filter->SetSize(out_size);

  auto old_origin = pointer_to_block_of_memory->GetOrigin() ;

  curan::renderable::Window::Info info;
  info.api_dump = false;
  info.display = "";
  info.full_screen = false;
  info.is_debug = false;
  info.screen_number = 0;
  info.title = "myviewer";
  curan::renderable::Window::WindowSize size_1{1000, 800};
  info.window_size = size_1;
  curan::renderable::Window window{info};

  curan::renderable::Volume::Info volumeinfo;
  volumeinfo.width = size.GetSize()[0];
  volumeinfo.height = size.GetSize()[1];
  volumeinfo.depth = size.GetSize()[2];
  volumeinfo.spacing_x = spacing[0];
  volumeinfo.spacing_y = spacing[1];
  volumeinfo.spacing_z = spacing[2];

  auto volume_to_render = curan::renderable::Volume::make(volumeinfo);
  window << volume_to_render;

  auto direction = pointer_to_block_of_memory->GetDirection();
    auto origin = pointer_to_block_of_memory->GetOrigin();
    auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            fixed_homogenenous_transformation(col, row) = direction(row, col);

    volume_to_render->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

    auto casted_volume_fixed = volume_to_render->cast<curan::renderable::Volume>();
    auto updater = [pointer_to_block_of_memory](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer_to_block_of_memory); };
    casted_volume_fixed->update_volume(updater);

  
  filter->SetOutputSpacing(new_spacing);
  itk::Matrix<double,3,3> new_direction;


  double angle = 0.0;
  new_direction(0,0) = std::cos(angle);
  new_direction(1,0) = std::sin(angle);
  new_direction(2,0) = 0.0;

  new_direction(0,1) = -std::sin(angle);
  new_direction(1,1) =  std::cos(angle);
  new_direction(2,1) = 0.0;

  new_direction(0,2) = 0.0;
  new_direction(1,2) = 0.0;
  new_direction(2,2) = 1.0;
  filter->SetOutputDirection(pointer_to_block_of_memory->GetDirection()*new_direction);

  
  TransformType::Pointer transform = TransformType::New();

  itk::Point<double, 3> rotation_center;
  rotation_center[0] = old_origin[0] + spacing[0] * size[0] / 2.0;
  rotation_center[1] = old_origin[1] + spacing[1] * size[1] / 2.0;
  rotation_center[2] = old_origin[2] + spacing[2] * size[2] / 2.0;

  transform->SetCenter(rotation_center);
  transform->SetRotation(0.0,0.0,0.0);

  TransformType::OutputVectorType translation;
  translation[0] = 0; // X translation in millimeters
  translation[1] = 0; // Y translation in millimeters
  translation[2] = 100; // Y translation in millimeters
  transform->SetTranslation(translation);
  

  itk::Point<double,3>::VectorType new_origin{{0.0,0.0,0.0}};
  old_origin += new_origin;
  filter->SetOutputOrigin(old_origin);


  TransformType::MatrixType matrix = transform->GetMatrix();
  TransformType::OffsetType offset = transform->GetOffset();

  std::cout << "transformation rotation matrix: \n" << matrix << std::endl;

  std::cout << "transformation translation vector: \n" << offset << "\n" << std::endl;
  
  filter->SetTransform(transform);



  try
  {
      filter->Update();
  }
  catch (const itk::ExceptionObject& e)
  {
      std::string result = "Failure to update the filter"+std::string{e.what()};
      std::cout << result;
      //return;
  }

  auto output = filter->GetOutput();

  auto render_transformation = vsg::translate(0.0, 0.0, 0.0);



  std::cout << "(output) origin : " << output->GetOrigin() << std::endl;
  std::cout << "(output) size : " << output->GetLargestPossibleRegion().GetSize() << std::endl;
  std::cout << "(output) spacing : " << output->GetSpacing() << std::endl;
  std::cout << "(output) direction : " << output->GetDirection() << std::endl;
 
  window.run();

  writer->SetInput(output);
  writer->Update();
 
  return EXIT_SUCCESS;
}