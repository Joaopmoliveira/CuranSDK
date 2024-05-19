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


constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;

using InputImageType = itk::Image<InputPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;

using Output2DImageType = itk::Image<OutputPixelType, Dimension_out>;

//using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;



using ReaderType = itk::ImageFileReader<InputImageType>;
using WriterType = itk::ImageFileWriter<Output2DImageType>;
 

 
int 
main(int argc, char * argv[]) {
 
  auto reader = ReaderType::New();
  auto writer = WriterType::New();
 
  std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
  std::string dirName_output{"extracted_slice.png"};

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

  using InterpolatorType = itk::LinearInterpolateImageFunction<InputImageType, double>;
  //using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
  auto interpolator = InterpolatorType::New();
  filter->SetInterpolator(interpolator);
  filter->SetDefaultPixelValue(0);

  auto input = pointer_to_block_of_memory;
  auto size = input->GetLargestPossibleRegion().GetSize();
  auto spacing = input->GetSpacing();
  double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);
  double maximum_size = std::max(std::max(size[0], size[1]), size[2]);
  auto new_spacing = spacing;
  new_spacing[0] = minimum_spacing;
  new_spacing[1] = minimum_spacing;
  new_spacing[2] = 100.0;

  auto out_size = size;
  out_size[0] = maximum_size;
  out_size[1] = maximum_size;
  out_size[2] = maximum_size;

  out_size[2] = 1;
  
  
  
  filter->SetInput(input);

  filter->SetSize(out_size);

  auto old_origin = pointer_to_block_of_memory->GetOrigin() ;

  
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

  TransformType::OutputVectorType translation;
  translation[0] = 0; // X translation in millimeters
  translation[1] = 0; // Y translation in millimeters
  translation[2] = 100; // Y translation in millimeters
  //transform->SetTranslation(translation);

  transform->SetOffset(translation);

  itk::Point<double, 3> rotation_center;
  rotation_center[0] = old_origin[0] + spacing[0] * size[0] / 2.0;
  rotation_center[1] = old_origin[1] + spacing[1] * size[1] / 2.0;
  rotation_center[2] = old_origin[2] + spacing[2] * size[2] / 2.0;

  transform->SetCenter(rotation_center);
  transform->SetRotation(3.14,0.0,0.0);

  
  

  itk::Point<double,3>::VectorType new_origin{{0.0,0.0,0.0}};
  old_origin += new_origin;
  filter->SetOutputOrigin(old_origin);


  TransformType::MatrixType matrix = transform->GetMatrix();
  TransformType::OffsetType offset = transform->GetOffset();

  std::cout << "transformation rotation matrix: \n" << matrix << std::endl;

  std::cout << "transformation translation vector: \n" << offset << "\n" << std::endl;


  std::cout << transform << std::endl;
  
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

  auto output1 = filter->GetOutput();

  using ExtractFilterType = itk::ExtractImageFilter<OutputImageType, Output2DImageType>;
  auto extractFilter = ExtractFilterType::New();
  extractFilter->SetDirectionCollapseToSubmatrix();

  // set up the extraction region [one slice]
  OutputImageType::RegionType inputRegion = output1->GetBufferedRegion();

  
  OutputImageType::SizeType sizee = inputRegion.GetSize();
  sizee[2] = 1; // we extract along z direction
  OutputImageType::IndexType start = inputRegion.GetIndex();
  const unsigned int sliceNumber = 0;
  start[2] = sliceNumber;
  OutputImageType::RegionType desiredRegion;
  desiredRegion.SetSize(sizee);
  desiredRegion.SetIndex(start);

  extractFilter->SetExtractionRegion(desiredRegion);

  std::cout << "\n ok \n" << std::endl;

  extractFilter->SetInput(output1);
  /* extractFilter->SetDirectionCollapseToSubmatrix();
  extractFilter->InPlaceOn();
   */
  try
  {
      extractFilter->Update();
  }
  catch (const itk::ExceptionObject& e)
  {
      std::string result = "Failure to update the filter "+std::string{e.what()};
      std::cout << result;
      //return;
  }

  auto output = extractFilter->GetOutput();

  /* std::cout << "(output) origin : " << output->GetOrigin() << std::endl;
  std::cout << "(output) size : " << output->GetLargestPossibleRegion().GetSize() << std::endl;
  std::cout << "(output) spacing : " << output->GetSpacing() << std::endl;
  std::cout << "(output) direction : " << output->GetDirection() << std::endl; */
 

  writer->SetInput(output);
  writer->Update();
 
  return EXIT_SUCCESS;
}