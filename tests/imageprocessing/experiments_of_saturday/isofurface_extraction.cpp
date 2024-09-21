#include <iostream>
#include <string>

#include "itkMeshSpatialObject.h"
#include "itkSpatialObjectReader.h"
#include "itkAutomaticTopologyMeshSource.h"
#include "imageprocessing/RegistrationUS_CT.h"
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
#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkCorrelationImageToImageMetricv4.h"
#include "itkJointHistogramMutualInformationImageToImageMetricv4.h"
#include "itkDemonsImageToImageMetricv4.h"
#include <itkANTSNeighborhoodCorrelationImageToImageMetricv4.h>
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMeshFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include <optional>
#include "itkBinaryMask3DMeshSource.h"
#include <type_traits>
#include <nlohmann/json.hpp>
#include <iostream>
#include "utils/TheadPool.h"
#include "itkRegionOfInterestImageFilter.h"
#include <itkTransformGeometryImageFilter.h>
#include <fstream>
#include <iomanip>
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkNormalVariateGenerator.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkImageMaskSpatialObject.h"
#include "itkThresholdImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkBinaryDilateImageFilter.h"
#include "itkEuclideanDistancePointMetric.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkPointSetToPointSetRegistrationMethod.h"
#include <random>
#include "itkVector.h"
#include "itkQuadEdgeMesh.h"
#include "itkMeshFileReader.h"
#include "itkQuadEdgeMeshExtendedTraits.h"
#include "itkNormalQuadEdgeMeshFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinomialBlurImageFilter.h"

template <typename T>
void update_ikt_filter(T &filter)
{
    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl
                  << err << std::endl;
        std::terminate();
    }
    catch (...)
    {
        std::cout << "generic unknown exception" << std::endl;
        std::terminate();
    }
}

int main(){
    auto image_reader = itk::ImageFileReader<itk::Image<double,3>>::New();
    //"C:/Dev/Curan/build/bin/resources/us_image1_cropepd_volume.mha"
    //"C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha"
    image_reader->SetFileName("C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha"); 
    auto bluring = itk::BinomialBlurImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
    bluring->SetInput(image_reader->GetOutput());
    bluring->SetRepetitions(8);

    update_ikt_filter(bluring);

    itk::Image<double,3>::Pointer image = bluring->GetOutput();

    constexpr double reduction_factor = 5.0;
    auto input_size = image->GetLargestPossibleRegion().GetSize();
    auto input_spacing = image->GetSpacing();
    auto input_origin = image->GetOrigin();

    std::cout << "input size: " << input_size << std::endl;
    std::cout << "input spacing: " << input_spacing << std::endl;

    double physicalspace[3];
    physicalspace[0] = input_size[0]*input_spacing[0];
    physicalspace[1] = input_size[1]*input_spacing[1];
    physicalspace[2] = input_size[2]*input_spacing[2];

    auto output_size = input_size;
    output_size[0] = (size_t)std::floor((1.0/reduction_factor)*output_size[0]);
    output_size[1] = (size_t)std::floor((1.0/reduction_factor)*output_size[1]);
    output_size[2] = (size_t)std::floor((1.0/reduction_factor)*output_size[2]);

    auto output_spacing = input_spacing;
    output_spacing[0] = physicalspace[0]/output_size[0];
    output_spacing[1] = physicalspace[1]/output_size[1];
    output_spacing[2] = physicalspace[2]/output_size[2];

    std::cout << "output size: " << output_size << std::endl;
    std::cout << "output spacing: " << output_spacing << std::endl;

    auto interpolator = itk::LinearInterpolateImageFunction<itk::Image<double,3>, double>::New();
    auto transform = itk::AffineTransform<double, 3>::New();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
    resampleFilter->SetInput(image);
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);

    {
        auto writer = itk::ImageFileWriter<itk::Image<double,3>>::New();
        writer->SetInput(resampleFilter->GetOutput());
        writer->SetFileName("processed.mha");
        update_ikt_filter(writer);   
    }


    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<itk::Image<double,3>>;
    using HistogramType = HistogramGeneratorType::HistogramType;
    auto histogramGenerator = HistogramGeneratorType::New();
    histogramGenerator->SetInput(resampleFilter->GetOutput());
    histogramGenerator->SetNumberOfBins(500);
    histogramGenerator->Compute();

    auto histogram = histogramGenerator->GetOutput();
    std::cout << "histogram size:" << histogram->Size() << std::endl;
    double total_frequency = 0;
    for (size_t i = 0; i < histogram->Size(); ++i)
        total_frequency += histogram->GetFrequency(i);

    auto target_frequency = 0.97 * total_frequency;

    std::cout << "target_frequency:" << target_frequency << std::endl;

    double cumulative_frequency = 0;
    size_t threshold_bin = 0;
    for (size_t i = 0; i < histogram->Size(); ++i){
        if (cumulative_frequency >= target_frequency){
            threshold_bin = i;
            std::cout << "stopping:" << i << std::endl;
            break;
        }
        std::printf("commulative: %f\n",cumulative_frequency);
        cumulative_frequency += histogram->GetFrequency(i);
    }
    // Calculate maximum value of the resampleFilter
    auto minMaxCalculator = itk::MinimumMaximumImageCalculator<itk::Image<double,3>>::New();
    minMaxCalculator->SetImage(resampleFilter->GetOutput());
    minMaxCalculator->Compute();


    std::printf("min value: %f max value: %f\n",minMaxCalculator->GetMinimum(),minMaxCalculator->GetMaximum());
    std::printf("min value: %f max value: %f\n",histogram->GetBinMin(0, threshold_bin),minMaxCalculator->GetMaximum()+1.0);

    HistogramType::MeasurementType thresholdvalue = histogram->GetBinMin(0, threshold_bin);

    auto binary_threshold = itk::BinaryThresholdImageFilter<itk::Image<double,3>, itk::Image<unsigned char,3>>::New();
    binary_threshold->SetInput(resampleFilter->GetOutput());
    binary_threshold->SetOutsideValue(0);
    binary_threshold->SetInsideValue(255);
    binary_threshold->SetLowerThreshold(histogram->GetBinMin(0, threshold_bin));
    binary_threshold->SetUpperThreshold(minMaxCalculator->GetMaximum()+1.0);

    auto meshSource = itk::BinaryMask3DMeshSource<itk::Image<unsigned char,3>, itk::Mesh<double>>::New();
    meshSource->SetObjectValue(1);
    meshSource->SetInput(binary_threshold->GetOutput());

    {
        auto writer = itk::ImageFileWriter<itk::Image<unsigned char,3>>::New();
        writer->SetInput(binary_threshold->GetOutput());
        writer->SetFileName("processed_filtered.mha");
        update_ikt_filter(writer);   
    }

    return 0;
}