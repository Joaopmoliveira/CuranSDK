#include "itkLaplacianImageFilter.h"
#include "itkZeroCrossingBasedEdgeDetectionImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImageDuplicator.h"
#include <optional>
#include <iostream>
#include <fstream>
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkVectorIndexSelectionCastImageFilter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"

#include "itkImageToListSampleAdaptor.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkHistogram.h"
#include "itkSampleToHistogramFilter.h"


using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using WriterType = itk::ImageFileWriter<ImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using LaplacianFilter = itk::LaplacianImageFilter<ImageType, ImageType>;
using FilterType1 = itk::RecursiveGaussianImageFilter<ImageType, ImageType>;
using DuplicatorType = itk::ImageDuplicator<ImageType>;
using FilterType3 = itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;

using VectorPixelType = itk::CovariantVector<float, Dimension>;
using VectorImageType = itk::Image<VectorPixelType, Dimension>;


int main(int argc, char **argv)
{
    if(argc!=3){
        std::cout << "To run the executable you must provide three arguments:\n "
                << "first parameter - input volume\n"
                << "second parameter - output volume \n";
        return 1;
    }
    
    auto fixedImageReader = FixedImageReaderType::New();

    std::string dirName{argv[1]};
    fixedImageReader->SetFileName(dirName);


    try
    {
        fixedImageReader->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();

 using FilterType10 =
itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;

auto laplacian = FilterType10::New();
laplacian->SetNormalizeAcrossScale(true);
laplacian->SetInput(pointer2fixedimage);
laplacian->SetSigma(10);

    std::string outputname{argv[2]};
    auto writer = WriterType::New();
    writer->SetFileName("2"+outputname);
     writer->SetInput(laplacian->GetOutput());
  try
  {
    writer->Update();
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}


int mumumummain(int argc, char **argv)
{
    if(argc!=3){
        std::cout << "To run the executable you must provide three arguments:\n "
                << "first parameter - input volume\n"
                << "second parameter - output volume \n";
        return 1;
    }
    
    auto fixedImageReader = FixedImageReaderType::New();

    std::string dirName{argv[1]};
    fixedImageReader->SetFileName(dirName);


    try
    {
        fixedImageReader->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();

  using FilterType4 = itk::VectorGradientAnisotropicDiffusionImageFilter<VectorImageType,VectorImageType>;
  FilterType4::Pointer filter = FilterType4::New();
  // Software Guide : EndCodeSnippet
  typedef itk::GradientRecursiveGaussianImageFilter<
                       ImageType, VectorImageType >   GradientFilterType;
  GradientFilterType::Pointer gradient = GradientFilterType::New();
  //  Software Guide : BeginLatex
  //
  //  The input image can be obtained from the output of another filter. Here,
  //  an image reader is used as source and its data is passed through a
  //  gradient filter in order to generate an image of vectors.
  //
  //  Software Guide : EndLatex
  // Software Guide : BeginCodeSnippet
  gradient->SetInput( pointer2fixedimage );
  filter->SetInput( gradient->GetOutput() );

  filter->SetNumberOfIterations( 40 );
  filter->SetTimeStep( 0.019);
  filter->SetConductanceParameter(1.0);
  filter->Update();
  typedef itk::VectorIndexSelectionCastImageFilter<
                  VectorImageType, ImageType > ComponentFilterType;
  ComponentFilterType::Pointer component = ComponentFilterType::New();
  // Select the component to extract.

{


component->SetIndex( 0 );
component->SetInput( filter->GetOutput() );
component->Update();

/*

using AdaptorType = itk::Statistics::ImageToListSampleAdaptor<ImageType>;
auto adaptor = AdaptorType::New();
adaptor->SetImage(component->GetOutput());

using HistogramMeasurementType = PixelType;
using HistogramType = itk::Statistics::Histogram<HistogramMeasurementType>;
using FilterType6 =
itk::Statistics::SampleToHistogramFilter<AdaptorType, HistogramType>;
auto filter6 = FilterType6::New();

constexpr unsigned int numberOfComponents = 1;
HistogramType::SizeType size(numberOfComponents);
size.Fill(255);
filter6->SetInput(adaptor);
filter6->SetHistogramSize(size);
filter6->SetMarginalScale(10);
HistogramType::MeasurementVectorType min(numberOfComponents);
HistogramType::MeasurementVectorType max(numberOfComponents);
min.Fill(-1.0);
max.Fill(10.0);
filter6->SetHistogramBinMinimum(min);
filter6->SetHistogramBinMaximum(max);
filter6->Update();

HistogramType::ConstPointer histogram = filter6->GetOutput();

const unsigned int histogramSize = histogram->Size();
std::cout << "Histogram size " << histogramSize << std::endl;
for (unsigned int bin = 0; bin < histogramSize; ++bin){
    std::cout << "bin = " << bin << " frequency = ";
    std::cout << histogram->GetFrequency(bin, 0) << std::endl;
}


*/
    std::string outputname{argv[2]};

    auto writer = WriterType::New();
    writer->SetFileName("0"+outputname);
     writer->SetInput(component->GetOutput());
  try
  {
    writer->Update();
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
  }
 }

 {


component->SetIndex( 1 );
component->SetInput( filter->GetOutput() );
component->Update();

/*

using AdaptorType = itk::Statistics::ImageToListSampleAdaptor<ImageType>;
auto adaptor = AdaptorType::New();
adaptor->SetImage(component->GetOutput());

using HistogramMeasurementType = PixelType;
using HistogramType = itk::Statistics::Histogram<HistogramMeasurementType>;
using FilterType6 =
itk::Statistics::SampleToHistogramFilter<AdaptorType, HistogramType>;
auto filter6 = FilterType6::New();

constexpr unsigned int numberOfComponents = 1;
HistogramType::SizeType size(numberOfComponents);
size.Fill(255);
filter6->SetInput(adaptor);
filter6->SetHistogramSize(size);
filter6->SetMarginalScale(10);
HistogramType::MeasurementVectorType min(numberOfComponents);
HistogramType::MeasurementVectorType max(numberOfComponents);
min.Fill(-1.0);
max.Fill(10.0);
filter6->SetHistogramBinMinimum(min);
filter6->SetHistogramBinMaximum(max);
filter6->Update();

HistogramType::ConstPointer histogram = filter6->GetOutput();

const unsigned int histogramSize = histogram->Size();
std::cout << "Histogram size " << histogramSize << std::endl;
for (unsigned int bin = 0; bin < histogramSize; ++bin){
    std::cout << "bin = " << bin << " frequency = ";
    std::cout << histogram->GetFrequency(bin, 0) << std::endl;
}


*/
    std::string outputname{argv[2]};
    auto writer = WriterType::New();
    writer->SetFileName("1"+outputname);
     writer->SetInput(component->GetOutput());
  try
  {
    writer->Update();
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
  }
 }

 {


component->SetIndex( 2 );
component->SetInput( filter->GetOutput() );
component->Update();

/*

using AdaptorType = itk::Statistics::ImageToListSampleAdaptor<ImageType>;
auto adaptor = AdaptorType::New();
adaptor->SetImage(component->GetOutput());

using HistogramMeasurementType = PixelType;
using HistogramType = itk::Statistics::Histogram<HistogramMeasurementType>;
using FilterType6 =
itk::Statistics::SampleToHistogramFilter<AdaptorType, HistogramType>;
auto filter6 = FilterType6::New();

constexpr unsigned int numberOfComponents = 1;
HistogramType::SizeType size(numberOfComponents);
size.Fill(255);
filter6->SetInput(adaptor);
filter6->SetHistogramSize(size);
filter6->SetMarginalScale(10);
HistogramType::MeasurementVectorType min(numberOfComponents);
HistogramType::MeasurementVectorType max(numberOfComponents);
min.Fill(-1.0);
max.Fill(10.0);
filter6->SetHistogramBinMinimum(min);
filter6->SetHistogramBinMaximum(max);
filter6->Update();

HistogramType::ConstPointer histogram = filter6->GetOutput();

const unsigned int histogramSize = histogram->Size();
std::cout << "Histogram size " << histogramSize << std::endl;
for (unsigned int bin = 0; bin < histogramSize; ++bin){
    std::cout << "bin = " << bin << " frequency = ";
    std::cout << histogram->GetFrequency(bin, 0) << std::endl;
}


*/
    std::string outputname{argv[2]};
    auto writer = WriterType::New();
    writer->SetFileName("2"+outputname);
     writer->SetInput(component->GetOutput());
  try
  {
    writer->Update();
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
  }
 }
  return EXIT_SUCCESS;
}