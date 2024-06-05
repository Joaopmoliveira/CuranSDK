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
#include "itkRelabelComponentImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkVectorIndexSelectionCastImageFilter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkThresholdImageFilter.h"
#include "itkImageToListSampleAdaptor.h"
#include "itkImage.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkImageFileReader.h"
#include "itkHistogram.h"
#include "itkSampleToHistogramFilter.h"
#include "itkMaskImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"

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

    using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale_filter = RescaleFilterType::New();
    rescale_filter->SetInput(fixedImageReader->GetOutput());
    rescale_filter->SetOutputMinimum(0.0);
    rescale_filter->SetOutputMaximum(1.0);


    try
    {
        rescale_filter->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = rescale_filter->GetOutput();

    using FilterType10 =
    itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;

    auto laplacian = FilterType10::New();
    laplacian->SetNormalizeAcrossScale(true);
    laplacian->SetInput(pointer2fixedimage);
    laplacian->SetSigma(1);
    laplacian->Update();

    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(laplacian->GetOutput());
    minMaxCalculator->Compute();

    PixelType minValue = minMaxCalculator->GetMinimum();
    PixelType maxValue = minMaxCalculator->GetMaximum();

    using ThresholdFilterType = itk::ThresholdImageFilter<ImageType>;
    ThresholdFilterType::Pointer thresholdFilter = ThresholdFilterType::New();
    thresholdFilter->SetInput(laplacian->GetOutput());
    thresholdFilter->ThresholdOutside(minValue, -0.02);
    thresholdFilter->SetOutsideValue(0);
    thresholdFilter->Update();

    std::cout << "Thresholding completed." << std::endl;

    using LabelType = unsigned short;
    using LabelImageType = itk::Image<LabelType, 3>;
    using ConnectedComponentFilterType = itk::ConnectedComponentImageFilter<ImageType, LabelImageType>;
    ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();
    connectedComponentFilter->SetInput(thresholdFilter->GetOutput());
    connectedComponentFilter->Update();

    std::cout << "Connected components filter completed." << std::endl;

    using RelabelFilterType = itk::RelabelComponentImageFilter<LabelImageType, LabelImageType>;
    RelabelFilterType::Pointer relabelFilter = RelabelFilterType::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
    relabelFilter->Update();

    std::cout << "Relabeling completed. Number of objects: " << relabelFilter->GetNumberOfObjects() << std::endl;

    using ThresholdFilterType2 = itk::ThresholdImageFilter<LabelImageType>;
    ThresholdFilterType2::Pointer thresholdFilter2 = ThresholdFilterType2::New();
    thresholdFilter2->SetInput(relabelFilter->GetOutput());
    thresholdFilter2->ThresholdOutside(1, 5); // Keep only the label 1 (largest component)
    thresholdFilter2->SetOutsideValue(0);
    thresholdFilter2->Update();

    LabelImageType::Pointer largestComponentMask = thresholdFilter2->GetOutput();
    std::cout << "Thresholding to keep largest component completed." << std::endl;

    using MaskFilterType = itk::MaskImageFilter<ImageType, LabelImageType, ImageType>;
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    maskFilter->SetInput(pointer2fixedimage);
    maskFilter->SetMaskImage(largestComponentMask);
    maskFilter->Update();

    std::string outputname{argv[2]};
    auto writer = WriterType::New();
    writer->SetFileName(outputname);
     writer->SetInput(maskFilter->GetOutput());
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