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
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkExtractImageFilter.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkLaplacianImageFilter.h"
#include "itkStatisticsImageFilter.h"
#include "itkScalarImageToHistogramGenerator.h"

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using WriterType = itk::ImageFileWriter<ImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using LaplacianFilter = itk::LaplacianImageFilter<ImageType, ImageType>;
using DuplicatorType = itk::ImageDuplicator<ImageType>;
using VectorPixelType = itk::CovariantVector<float, Dimension>;
using VectorImageType = itk::Image<VectorPixelType, Dimension>;
using GaussianFilterType = itk::SmoothingRecursiveGaussianImageFilter<ImageType, ImageType>;


int main(int argc, char **argv)
{
    if(argc!=4){
        std::cout << "To run the executable you must provide three arguments:\n "
                << "first parameter - input volume\n"
                << "second parameter - output volume \n"
                << "third parameter - sigma \n";
        return 1;
    }
    
    auto fixedImageReader = FixedImageReaderType::New();
    float sigma = std::atof(argv[3]);
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

    auto gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(pointer2fixedimage);
    gaussianFilter->SetSigma(sigma);

    auto writer = WriterType::New();
    writer->SetInput(gaussianFilter->GetOutput());
    writer->SetFileName("Gaussian_filtered.mha");
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

    using FilterType10 = itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;
    auto laplacian = FilterType10::New();
    laplacian->SetNormalizeAcrossScale(true);
    laplacian->SetInput(gaussianFilter->GetOutput());
    laplacian->Update();

    writer->SetInput(laplacian->GetOutput());
    writer->SetFileName("Laplacian.mha");

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

    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(laplacian->GetOutput());
    minMaxCalculator->Compute();

    PixelType minValue = minMaxCalculator->GetMinimum();

    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<ImageType>;
    using HistogramType = HistogramGeneratorType::HistogramType;

    auto histogramGenerator = HistogramGeneratorType::New();
    histogramGenerator->SetInput(laplacian->GetOutput());
    histogramGenerator->SetNumberOfBins(500); 

    try
    {
        histogramGenerator->Compute();
    }
    catch (itk::ExceptionObject & error)
    {
        std::cerr << "Error computing histogram: " << error << std::endl;
        return EXIT_FAILURE;
    }

    using HistogramType = HistogramGeneratorType::HistogramType;
    const HistogramType * histogram = histogramGenerator->GetOutput();

    int total_frequency = 0;
    for (unsigned int i = 0; i < histogram->Size(); ++i)
    {
        total_frequency += histogram->GetFrequency(i);
    }

    auto target_frequency = 0.1 * total_frequency;

    unsigned int cumulative_frequency = 0;
    unsigned int threshold_bin = 0;

    for (unsigned int i = 0; i < histogram->Size(); ++i)
    {
        cumulative_frequency += histogram->GetFrequency(i);
        if (cumulative_frequency >= target_frequency)
        {
            threshold_bin = i;
            break;
        }
    }

    HistogramType::MeasurementType thresholdvalue = histogram->GetBinMin(0, threshold_bin);
    

    using ThresholdFilterType = itk::ThresholdImageFilter<ImageType>;
    ThresholdFilterType::Pointer thresholdFilter = ThresholdFilterType::New();
    thresholdFilter->SetInput(laplacian->GetOutput());
    thresholdFilter->ThresholdOutside(minValue, thresholdvalue); //0.02 para a precious
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
    thresholdFilter2->ThresholdOutside(1, 2); // Keep only the label 1 (largest component) for US and 5 for ultrasound
    thresholdFilter2->SetOutsideValue(0);
    thresholdFilter2->Update();

    LabelImageType::Pointer largestComponentMask = thresholdFilter2->GetOutput();
    std::cout << "Thresholding to keep largest component completed." << std::endl;

    using MaskFilterType = itk::MaskImageFilter<ImageType, LabelImageType, ImageType>;
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    maskFilter->SetInput(pointer2fixedimage);
    maskFilter->SetMaskImage(largestComponentMask);
    maskFilter->Update();

   

  using ConstIteratorType = itk::ImageRegionConstIterator<LabelImageType>;
  ConstIteratorType inputIt(thresholdFilter2->GetOutput(), thresholdFilter2->GetOutput()->GetRequestedRegion());
  
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  
  std::pair<size_t,size_t> xlimits = {std::numeric_limits<size_t>::max(),0}; 
  std::pair<size_t,size_t> ylimits = {std::numeric_limits<size_t>::max(),0}; 
  std::pair<size_t,size_t> zlimits = {std::numeric_limits<size_t>::max(),0}; 
  while (!inputIt.IsAtEnd())
  {

    if(inputIt.Get()>0){
      const auto& index = inputIt.GetIndex();
      if(xlimits.first>index[0] ){
        xlimits.first = index[0];
      }
      if(xlimits.second<index[0] ){
        xlimits.second = index[0];
      }
      if(ylimits.first>index[1] ){
        ylimits.first = index[1];
      }
      if(ylimits.second<index[1] ){
        ylimits.second = index[1];
      }
      if(zlimits.first>index[2] ){
        zlimits.first = index[2];
      }
      if(zlimits.second<index[2] ){
        zlimits.second = index[2];
      }
    }
    ++inputIt;
  }
    
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
  std::printf("The limits of the image are: \ndirection 0 (%llu %llu / %llu)\ndirection 1 (%llu %llu / %llu)\ndirection 2 (%llu %llu / %llu)\n",xlimits.first,xlimits.second,maskFilter->GetOutput()->GetRequestedRegion().GetSize()[0],ylimits.first,ylimits.second,maskFilter->GetOutput()->GetRequestedRegion().GetSize()[1],zlimits.first,zlimits.second,maskFilter->GetOutput()->GetRequestedRegion().GetSize()[2]);
  
  using FilterType = itk::ExtractImageFilter<ImageType, ImageType>;
  FilterType::Pointer filter = FilterType::New();
  filter->SetInput(maskFilter->GetOutput());
  ImageType::RegionType inputRegion = maskFilter->GetOutput()->GetLargestPossibleRegion();
  ImageType::SizeType size = inputRegion.GetSize();
  size[0] = xlimits.second - xlimits.first + 1;
  size[1] = ylimits.second - ylimits.first + 1;
  size[2] = zlimits.second - zlimits.first + 1;
  ImageType::IndexType start = inputRegion.GetIndex();
  start[0] = xlimits.first;
  start[1] = ylimits.first;
  start[2] = zlimits.first;
  ImageType::RegionType desiredRegion;
  desiredRegion.SetSize(size);
  desiredRegion.SetIndex(start);
  
    std::string outputname{argv[2]};
    //auto writer = WriterType::New();
  filter->SetExtractionRegion(desiredRegion);
  writer->SetInput(filter->GetOutput());
  writer->SetFileName(outputname);
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