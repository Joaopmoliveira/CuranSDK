#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>
#include "utils/TheadPool.h"
#include "utils/Reader.h"
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Volume.h"
#include "rendering/Box.h"
#include "rendering/Sphere.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "imgui_stdlib.h"
#include <map>
#include <string>
#include "utils/FileStructures.h"
#include "robotutils/RobotModel.h"
#include "utils/DateManipulation.h"

/*
This executable requires:

1. Temporal calibration has been performed on the robot
which translates into a file called temporal_calibration.json
with the date at which the calibration was performed

2. Spatial calibration has been performed on the robot
which translates into a file called spatial_calibration.json
with the date at which the calibration was performed

And it outputs

1.

*/

void convert_robot_confi(const curan::robotic::RobotModel<curan::robotic::number_of_joints>& robot,igtl::Matrix4x4& matrix)
{
	matrix[0][0] = robot.rotation()(0,0);
	matrix[1][0] = robot.rotation()(1,0);
	matrix[2][0] = robot.rotation()(2,0);

	matrix[0][1] = robot.rotation()(0,1);
	matrix[1][1] = robot.rotation()(1,1);
	matrix[2][1] = robot.rotation()(2,1);

	matrix[0][2] = robot.rotation()(0,2);
	matrix[1][2] = robot.rotation()(1,2);
	matrix[2][2] = robot.rotation()(2,2);

	matrix[3][0] = 0.0;
	matrix[3][1] = 0.0;
	matrix[3][2] = 0.0;
	matrix[3][3] = 1.0;

	matrix[0][3] = robot.translation()[0]*1.0e3;
	matrix[1][3] = robot.translation()[1]*1.0e3;
	matrix[2][3] = robot.translation()[2]*1.0e3;
	return;
}

template <typename itkImage>
void updateBaseTexture3D(vsg::floatArray3D &image,typename itkImage::Pointer out) {
using OutputPixelType = double;
using OutputImageType = itk::Image<OutputPixelType, 3>;
using FilterType = itk::CastImageFilter<itkImage, OutputImageType>;
auto filter = FilterType::New();
filter->SetInput(out);

using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
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
  itk::ImageRegionIteratorWithIndex<OutputImageType> outputIt(rescale->GetOutput(), rescale->GetOutput()->GetRequestedRegion());
  for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    image.set(outputIt.GetIndex()[0], outputIt.GetIndex()[1],outputIt.GetIndex()[2], outputIt.Get());
  image.dirty();
}

using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<OutputPixelType, 3>;

using PixelType = float;
using ImageType = itk::Image<PixelType, 3>;

ImageType::Pointer DeepCopy(ImageType::Pointer input)
{
    ImageType::Pointer output = ImageType::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<ImageType> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<ImageType> outputIterator(output, output->GetLargestPossibleRegion());

    for(;!inputIterator.IsAtEnd(); ++inputIterator,++outputIterator)
            outputIterator.Set(inputIterator.Get());
    
    return output;
}

#include <iostream>
#include <string>

#include "imageprocessing/VolumetricRegistration.h"
#include "itkAutomaticTopologyMeshSource.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkBinaryDilateImageFilter.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinomialBlurImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCenteredTransformInitializer.h"
#include "itkCommand.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkCorrelationImageToImageMetricv4.h"
#include "itkDemonsImageToImageMetricv4.h"
#include "itkEuclideanDistancePointMetric.h"
#include "itkEuler3DTransform.h"
#include "itkExtractImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkHistogram.h"
#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkImageMaskSpatialObject.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkImageSeriesReader.h"
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkImageToListSampleAdaptor.h"
#include "itkJointHistogramMutualInformationImageToImageMetricv4.h"
#include "itkLaplacianImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkMaskImageFilter.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkMeshSpatialObject.h"
#include "itkNormalQuadEdgeMeshFilter.h"
#include "itkNormalVariateGenerator.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkPointSetToPointSetRegistrationMethod.h"
#include "itkQuadEdgeMesh.h"
#include "itkQuadEdgeMeshExtendedTraits.h"
#include "itkRegionOfInterestImageFilter.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkRelabelComponentImageFilter.h"
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkSampleToHistogramFilter.h"
#include "itkScalarImageToHistogramGenerator.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkSpatialObjectReader.h"
#include "itkStatisticsImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkThresholdImageFilter.h"
#include "itkVector.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorIndexSelectionCastImageFilter.h"
#include "itkVersorRigid3DTransform.h"
#include "itkZeroCrossingBasedEdgeDetectionImageFilter.h"
#include "utils/TheadPool.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <itkANTSNeighborhoodCorrelationImageToImageMetricv4.h>
#include <itkTransformGeometryImageFilter.h>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <type_traits>

template <typename T> 
void update_ikt_filter(T &filter) {
    try {
      filter->Update();
    } catch (const itk::ExceptionObject &err) {
      std::cout << "ExceptionObject caught !" << std::endl << err << std::endl;
      std::terminate();
    } catch (...) {
      std::cout << "generic unknown exception" << std::endl;
      std::terminate();
    }
  }
  
  inline double rad2deg(double in) {
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians * in;
  }
  
  // Lamda to get a rotation matrix from a given angle
  auto transform_x = [](double alpha) {
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << 1.0, 0.0, 0.0, 0.0, std::cos(alpha),
        -std::sin(alpha), 0.0, std::sin(alpha), std::cos(alpha);
    Eigen::Matrix<double, 4, 4> T_rotation_extra =
        Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
  };
  
  auto transform_y = [](double alpha) {
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << std::cos(alpha), 0.0, std::sin(alpha), 0.0,
        1.0, 0.0, -std::sin(alpha), 0.0, std::cos(alpha);
    Eigen::Matrix<double, 4, 4> T_rotation_extra =
        Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
  };
  
  auto transform_z = [](double alpha) {
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << std::cos(alpha), -std::sin(alpha), 0.0,
        std::sin(alpha), std::cos(alpha), 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 4, 4> T_rotation_extra =
        Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
  };
  
  Eigen::Matrix<double, 1, 9>
get_error(Eigen::Matrix<double, 4, 4> moving_to_fixed) {
    Eigen::Matrix<double, 3, 9> ct_points;
    ct_points << 51.0910, 45.7005, 18.7328, -26.3736, -53.1790, -53.1238,
        -38.7812, 5.9589, -26.4164, 80.3660, 128.6975, 156.8359, 158.2443,
        127.2275, 87.3330, 67.2878, 99.4279, 129.4901, 517.6372, 516.1141,
        514.3752, 514.4684, 515.4078, 517.3948, 519.6219, 505.9474, 509.5589;
  
    Eigen::Matrix<double, 3, 9> world_points;
    world_points << -674.874, -714.276, -741.764, -741.121, -716.382, -677.195,
        -660.926, -683.793, -713.982, 90.9651, 88.4241, 58.843, 15.514, -11.3519,
        -9.55646, 3.30793, 50.3275, 16.0374, 31.3693, 8.2447, -2.56189, -5.17164,
        11.5598, 26.49, 36.5624, 10.8589, 2.87831;
  
    Eigen::Matrix<double, 3, 9> moved =
        (moving_to_fixed.block<3, 3>(0, 0) * ct_points).colwise() +
        moving_to_fixed.block<3, 1>(0, 3);
  
    Eigen::Matrix<double, 3, 9> error = world_points - moved;
    Eigen::Array<double, 1, 9> rooted =
        error.array().square().colwise().sum().sqrt();
    return rooted.matrix();
  }
  
  // Lamda to get a rotation matrix from a given angle but with a flipped
  // principal direction
  auto transform_flipped_principal_component = [](double alpha) {
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << 1.0, 0.0, 0.0, 0.0, std::cos(alpha),
        -std::sin(alpha), 0.0, std::sin(alpha), std::cos(alpha);
  
    Eigen::Matrix<double, 3, 3> flipping_direction;
    flipping_direction << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  
    Eigen::Matrix<double, 4, 4> T_rotation_extra =
        Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) =
        poorly_constrained_direction * flipping_direction;
    return T_rotation_extra;
  };
  
  enum ExtractionSurfaceTransformation {
    ELIMINATE_VERTICIES_WITH_OUTWARD_NORMALS,
    ELIMINATE_VERTICIES_WITH_INNARD_NORMALS,
    SUBSAMPLE_VERTCIES,
    LEAVE_UNCHANGED
  };
  
  template <bool is_in_debug> struct ExtractionSurfaceInfo {
    size_t connected_components;
    double frequency;
    std::string appendix;
    double reduction_factor;
    ExtractionSurfaceTransformation transform_surface = LEAVE_UNCHANGED;
    int buffer_of_mask;
  
    ExtractionSurfaceInfo(size_t in_connected_components, double in_frequency,
                          std::string in_appendix, double in_reduction_factor,
                          int in_buffer_of_mask)
        : connected_components{in_connected_components}, frequency{in_frequency},
          appendix{in_appendix}, reduction_factor{in_reduction_factor},
          buffer_of_mask{in_buffer_of_mask} {};
  };
  
  template <bool is_in_debug>
  std::tuple<itk::Mesh<double>::Pointer,itk::ImageMaskSpatialObject<3>::ImageType::Pointer> extract_point_cloud(itk::Image<float, 3>::Pointer image, const ExtractionSurfaceInfo<is_in_debug> &info) {
    auto image_to_fill = itk::ImageMaskSpatialObject<3>::ImageType::New();
  
    auto bluring = itk::BinomialBlurImageFilter<itk::Image<float, 3>,itk::Image<float, 3>>::New();
    bluring->SetInput(image);
    bluring->SetRepetitions(10);
  
    update_ikt_filter(bluring);
  
    auto input_size = image->GetLargestPossibleRegion().GetSize();
    auto input_spacing = image->GetSpacing();
    auto input_origin = image->GetOrigin();
  
    double physicalspace[3];
    physicalspace[0] = input_size[0] * input_spacing[0];
    physicalspace[1] = input_size[1] * input_spacing[1];
    physicalspace[2] = input_size[2] * input_spacing[2];
  
    auto output_size = input_size;
    output_size[0] = (size_t)std::floor((1.0 / info.reduction_factor) * output_size[0]);
    output_size[1] = (size_t)std::floor((1.0 / info.reduction_factor) * output_size[1]);
    output_size[2] = (size_t)std::floor((1.0 / info.reduction_factor) * output_size[2]);
  
    auto output_spacing = input_spacing;
    output_spacing[0] = physicalspace[0] / output_size[0];
    output_spacing[1] = physicalspace[1] / output_size[1];
    output_spacing[2] = physicalspace[2] / output_size[2];
  
    auto interpolator =
        itk::LinearInterpolateImageFunction<itk::Image<float, 3>, double>::New();
    auto transform = itk::AffineTransform<double, 3>::New();
    transform->SetIdentity();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<float, 3>,
                                                   itk::Image<float, 3>>::New();
    resampleFilter->SetInput(bluring->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetOutputDirection(bluring->GetOutput()->GetDirection());
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);
  
    if constexpr (is_in_debug) {
      {
        auto writer = itk::ImageFileWriter<itk::Image<float, 3>>::New();
        writer->SetInput(resampleFilter->GetOutput());
        writer->SetFileName(info.appendix + "_processed.mha");
        update_ikt_filter(writer);
      }
      {
        auto writer = itk::ImageFileWriter<itk::Image<float, 3>>::New();
        writer->SetInput(resampleFilter->GetOutput());
        writer->SetFileName(info.appendix + "_bluered.mha");
        update_ikt_filter(writer);
      }
    } else
      update_ikt_filter(resampleFilter);
  
    using HistogramGeneratorType =
        itk::Statistics::ScalarImageToHistogramGenerator<itk::Image<float, 3>>;
    using HistogramType = HistogramGeneratorType::HistogramType;
    auto histogramGenerator = HistogramGeneratorType::New();
    histogramGenerator->SetInput(resampleFilter->GetOutput());
    histogramGenerator->SetNumberOfBins(500);
    histogramGenerator->Compute();
  
    auto histogram = histogramGenerator->GetOutput();
    double total_frequency = 0;
    for (size_t i = 1; i < histogram->Size(); ++i)
      total_frequency += histogram->GetFrequency(i);
  
    auto target_frequency = info.frequency * total_frequency;
  
    double cumulative_frequency = 0;
    size_t threshold_bin = 0;
    for (size_t i = 1; i < histogram->Size(); ++i) {
      if (cumulative_frequency >= target_frequency) {
        threshold_bin = i;
        break;
      }
      cumulative_frequency += histogram->GetFrequency(i);
    }
    auto minMaxCalculator =
        itk::MinimumMaximumImageCalculator<itk::Image<float, 3>>::New();
    minMaxCalculator->SetImage(resampleFilter->GetOutput());
    minMaxCalculator->Compute();
  
    HistogramType::MeasurementType thresholdvalue =
        histogram->GetBinMin(0, threshold_bin);
  
    auto binary_threshold =
        itk::BinaryThresholdImageFilter<itk::Image<float, 3>,
                                        itk::Image<unsigned char, 3>>::New();
    binary_threshold->SetInput(resampleFilter->GetOutput());
    binary_threshold->SetOutsideValue(0);
    binary_threshold->SetInsideValue(255);
    binary_threshold->SetLowerThreshold(histogram->GetBinMin(0, threshold_bin));
    binary_threshold->SetUpperThreshold(minMaxCalculator->GetMaximum() + 1.0);
  
    auto connectedComponentFilter =
        itk::ConnectedComponentImageFilter<itk::Image<unsigned char, 3>,
                                           itk::Image<unsigned char, 3>>::New();
    connectedComponentFilter->SetInput(binary_threshold->GetOutput());
  
    auto relabelFilter =
        itk::RelabelComponentImageFilter<itk::Image<unsigned char, 3>,
                                         itk::Image<unsigned char, 3>>::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
  
    auto filtered_image_with_largest_components =
        itk::ThresholdImageFilter<itk::Image<unsigned char, 3>>::New();
    filtered_image_with_largest_components->SetInput(relabelFilter->GetOutput());
    filtered_image_with_largest_components->ThresholdOutside(
        1, info.connected_components);
    filtered_image_with_largest_components->SetOutsideValue(255);
  
    auto final_binary_threshold =
        itk::BinaryThresholdImageFilter<itk::Image<unsigned char, 3>,
                                        itk::Image<unsigned char, 3>>::New();
    final_binary_threshold->SetInput(
        filtered_image_with_largest_components->GetOutput());
    final_binary_threshold->SetOutsideValue(0);
    final_binary_threshold->SetInsideValue(255);
    final_binary_threshold->SetLowerThreshold(0);
    final_binary_threshold->SetUpperThreshold(info.connected_components);
  
    if constexpr (is_in_debug) {
      auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
      writer->SetInput(final_binary_threshold->GetOutput());
      writer->SetFileName(info.appendix + "_processed_filtered.mha");
      update_ikt_filter(writer);
    } else
      update_ikt_filter(final_binary_threshold);
  
    // we extract the mask as a function of the number of connected components we
    // wish to extract
    {
      std::vector<int> minimum_x_indicies_of_regions;
      minimum_x_indicies_of_regions.resize(info.connected_components);
      std::vector<int> maximum_x_indicies_of_regions;
      maximum_x_indicies_of_regions.resize(info.connected_components);
      std::vector<int> minimum_y_indicies_of_regions;
      minimum_y_indicies_of_regions.resize(info.connected_components);
      std::vector<int> maximum_y_indicies_of_regions;
      maximum_y_indicies_of_regions.resize(info.connected_components);
      std::vector<int> minimum_z_indicies_of_regions;
      minimum_z_indicies_of_regions.resize(info.connected_components);
      std::vector<int> maximum_z_indicies_of_regions;
      maximum_z_indicies_of_regions.resize(info.connected_components);
      for (unsigned char highlighted_region = 0;
           highlighted_region < info.connected_components; ++highlighted_region) {
        minimum_x_indicies_of_regions[highlighted_region] = 10000;
        maximum_x_indicies_of_regions[highlighted_region] = 0;
        minimum_y_indicies_of_regions[highlighted_region] = 10000;
        maximum_y_indicies_of_regions[highlighted_region] = 0;
        minimum_z_indicies_of_regions[highlighted_region] = 10000;
        maximum_z_indicies_of_regions[highlighted_region] = 0;
      }
  
      itk::ImageRegionIteratorWithIndex<itk::Image<unsigned char, 3>> iterator(
          relabelFilter->GetOutput(),
          relabelFilter->GetOutput()->GetLargestPossibleRegion());
      iterator.GoToBegin();
  
      while (!iterator.IsAtEnd()) {
        for (int highlighted_region = 0;
             highlighted_region < info.connected_components;
             ++highlighted_region) {
          if ((int)iterator.Get() == (highlighted_region + 1)) {
            auto index = iterator.GetIndex();
            if (minimum_x_indicies_of_regions[highlighted_region] > index[0])
              minimum_x_indicies_of_regions[highlighted_region] = index[0];
            if (maximum_x_indicies_of_regions[highlighted_region] < index[0])
              maximum_x_indicies_of_regions[highlighted_region] = index[0];
            if (minimum_y_indicies_of_regions[highlighted_region] > index[1])
              minimum_y_indicies_of_regions[highlighted_region] = index[1];
            if (maximum_y_indicies_of_regions[highlighted_region] < index[1])
              maximum_y_indicies_of_regions[highlighted_region] = index[1];
            if (minimum_z_indicies_of_regions[highlighted_region] > index[2])
              minimum_z_indicies_of_regions[highlighted_region] = index[2];
            if (maximum_z_indicies_of_regions[highlighted_region] < index[2])
              maximum_z_indicies_of_regions[highlighted_region] = index[2];
          }
        }
        ++iterator;
      }
  
      itk::ImageMaskSpatialObject<3>::ImageType::SizeType size =
          relabelFilter->GetOutput()->GetLargestPossibleRegion().GetSize();
      itk::ImageMaskSpatialObject<3>::ImageType::IndexType index = {{0, 0, 0}};
      itk::ImageMaskSpatialObject<3>::ImageType::RegionType region;
      region.SetSize(size);
      region.SetIndex(index);
      image_to_fill->SetRegions(region);
      image_to_fill->Allocate(true);
      image_to_fill->SetSpacing(relabelFilter->GetOutput()->GetSpacing());
      image_to_fill->SetOrigin(relabelFilter->GetOutput()->GetOrigin());
      image_to_fill->SetDirection(relabelFilter->GetOutput()->GetDirection());
      itk::ImageRegionIteratorWithIndex<itk::Image<unsigned char, 3>>
          iterator_of_image_to_fill(image_to_fill,
                                    image_to_fill->GetLargestPossibleRegion());
      while (!iterator_of_image_to_fill.IsAtEnd()) {
        auto index = iterator_of_image_to_fill.GetIndex();
        for (unsigned char highlighted_region = 0;
             highlighted_region < info.connected_components;
             ++highlighted_region) {
          bool is_inside_highlighted =
              (index[0] > minimum_x_indicies_of_regions[highlighted_region] -
                              info.buffer_of_mask &&
               index[0] <
                   info.buffer_of_mask +
                       maximum_x_indicies_of_regions[highlighted_region]) &&
              (index[1] > minimum_y_indicies_of_regions[highlighted_region] -
                              info.buffer_of_mask &&
               index[1] <
                   info.buffer_of_mask +
                       maximum_y_indicies_of_regions[highlighted_region]) &&
              (index[2] > minimum_z_indicies_of_regions[highlighted_region] -
                              info.buffer_of_mask &&
               index[2] < info.buffer_of_mask +
                              maximum_z_indicies_of_regions[highlighted_region]);
          if (is_inside_highlighted) {
            iterator_of_image_to_fill.Set(255);
            break;
          }
        }
        ++iterator_of_image_to_fill;
      }
  
      if constexpr (is_in_debug) {
        auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
        writer->SetInput(image_to_fill);
        writer->SetFileName(info.appendix + "_filled_mask.mha");
        update_ikt_filter(writer);
      }
    }
  
    auto meshSource = itk::BinaryMask3DMeshSource<itk::Image<unsigned char, 3>,
                                                  itk::Mesh<double>>::New();
    meshSource->SetObjectValue(255);
    meshSource->SetInput(final_binary_threshold->GetOutput());
    update_ikt_filter(meshSource);
  
    if constexpr (is_in_debug) {
      using WriterType = itk::MeshFileWriter<itk::Mesh<double>>;
      auto writer = WriterType::New();
      writer->SetFileName(info.appendix + "_point_cloud.obj");
      writer->SetInput(meshSource->GetOutput());
      update_ikt_filter(writer);
    }
  
    return {meshSource->GetOutput(), image_to_fill};
  }
  
  itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::Pointer
  convert_mesh(const itk::Mesh<double>::Pointer &mesh) {
    class MeshTriangleVisitor {
    public:
      itk::Mesh<double>::Pointer inmesh;
      using MeshSourceType = itk::AutomaticTopologyMeshSource<itk::Mesh<double>>;
      itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer container;
  
      void set_required_data(itk::Mesh<double>::Pointer ref_inmesh) {
        inmesh = ref_inmesh;
        container = itk::QuadEdgeMesh<double, 3>::PointsContainer::New();
        container->Reserve(inmesh->GetNumberOfPoints());
      }
  
      using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
      void Visit(unsigned long cellId, TriangleType *t) {
        TriangleType::PointIdIterator pit = t->PointIdsBegin();
        for (size_t index = 0; pit != t->PointIdsEnd(); ++pit, ++index) {
          container->SetElement(*pit, inmesh->GetPoint(*pit));
        }
      }
  
      MeshTriangleVisitor() = default;
      virtual ~MeshTriangleVisitor() = default;
    };
  
    class MeshTriangleConstructorVisitor {
    public:
      itk::Mesh<double>::Pointer inmesh;
      using MeshSourceType = itk::AutomaticTopologyMeshSource<itk::Mesh<double>>;
      itk::QuadEdgeMesh<double, 3>::Pointer outmesh;
  
      void set_required_data(
          itk::Mesh<double>::Pointer ref_inmesh,
          itk::QuadEdgeMesh<double, 3>::Pointer ref_outmesh,
          itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer in_container) {
        inmesh = ref_inmesh;
        outmesh = ref_outmesh;
        outmesh->SetPoints(in_container);
      }
  
      using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
      void Visit(unsigned long cellId, TriangleType *t) {
        TriangleType::PointIdIterator pit = t->PointIdsBegin();
        std::array<TriangleType::PointIdentifier, 3> ident;
        for (size_t index = 0; pit != t->PointIdsEnd(); ++pit, ++index)
          ident[index] = *pit;
        outmesh->AddFaceTriangle(ident[0], ident[1], ident[2]);
      }
  
      MeshTriangleConstructorVisitor() = default;
      virtual ~MeshTriangleConstructorVisitor() = default;
    };
  
    itk::QuadEdgeMesh<double, 3>::Pointer out_mesh =
        itk::QuadEdgeMesh<double, 3>::New();
    itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer container;
    {
      using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
      using TriangleVisitorInterfaceType =
          itk::CellInterfaceVisitorImplementation<
              itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits,
              TriangleType, MeshTriangleVisitor>;
      auto triangleVisitor = TriangleVisitorInterfaceType::New();
      triangleVisitor->set_required_data(mesh);
      using CellMultiVisitorType = itk::Mesh<double>::CellType::MultiVisitor;
      auto multiVisitor = CellMultiVisitorType::New();
      multiVisitor->AddVisitor(triangleVisitor);
      mesh->Accept(multiVisitor);
      container = triangleVisitor->container;
    }
    {
      using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
      using TriangleVisitorInterfaceType =
          itk::CellInterfaceVisitorImplementation<
              itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits,
              TriangleType, MeshTriangleConstructorVisitor>;
      auto triangleVisitor = TriangleVisitorInterfaceType::New();
      triangleVisitor->set_required_data(mesh, out_mesh, container);
      using CellMultiVisitorType = itk::Mesh<double>::CellType::MultiVisitor;
      auto multiVisitor = CellMultiVisitorType::New();
      multiVisitor->AddVisitor(triangleVisitor);
      mesh->Accept(multiVisitor);
    }
    auto computation_of_normals = itk::NormalQuadEdgeMeshFilter<
        itk::QuadEdgeMesh<double, 3>,
        itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::New();
    itk::NormalQuadEdgeMeshFilter<
        itk::QuadEdgeMesh<double, 3>,
        itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::WeightEnum weight_type =
        itk::NormalQuadEdgeMeshFilterEnums::Weight::THURMER;
    computation_of_normals->SetInput(out_mesh);
    computation_of_normals->SetWeight(weight_type);
    update_ikt_filter(computation_of_normals);
    return computation_of_normals->GetOutput();
  }
  
  itk::PointSet<double, 3>::Pointer
  prune_surface(itk::Mesh<double>::Pointer mesh) {
    auto converted_mesh = convert_mesh(mesh);
    auto point_container = itk::PointSet<double, 3>::PointsContainer::New();
    itk::PointSet<double, 3>::Pointer point_set = itk::PointSet<double, 3>::New();
    itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::PointsContainerIterator p_it =
        converted_mesh->GetPoints()->Begin();
    itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::PointDataContainerIterator
        d_it = converted_mesh->GetPointData()->Begin();
    size_t index = 0;
    while (p_it != converted_mesh->GetPoints()->End()) {
      Eigen::Matrix<double, 3, 1> normal;
      normal[0] = d_it.Value()[0];
      normal[1] = d_it.Value()[1];
      normal[2] = d_it.Value()[2];
      Eigen::Matrix<double, 3, 1> point;
      point[0] = p_it.Value()[0];
      point[1] = p_it.Value()[1];
      point[2] = p_it.Value()[2];
      normal.normalize();
      point.normalize();
      if (normal.transpose() * point > 0.23) {
        point_container->InsertElement(index, p_it->Value());
        ++index;
      }
      ++p_it;
      ++d_it;
    }
    std::cout << "Number of points after prunning :" << index << std::endl;
    point_set->SetPoints(point_container);
    return point_set;
  }
  
  std::tuple<Eigen::Matrix<double, 4, 4>, itk::PointSet<double, 3>::Pointer>
  recentered_data(const itk::Mesh<double>::Pointer &mesh) {
    std::cout << "Number of points :" << mesh->GetNumberOfPoints() << std::endl;
    Eigen::Matrix<double, 4, 4> pca_alignement =
        Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, 3> points_in_matrix_form =
        Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(mesh->GetNumberOfPoints(),
                                                       3);
    using PointsIterator = itk::Mesh<double>::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    size_t index = 0;
    while (pointIterator != mesh->GetPoints()->End()) {
      auto p = pointIterator->Value();
      points_in_matrix_form(index, 0) = p[0];
      points_in_matrix_form(index, 1) = p[1];
      points_in_matrix_form(index, 2) = p[2];
      ++pointIterator;
      ++index;
    }
  
    Eigen::Matrix<double, 3, 1> center_of_moving_image =
        points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double, 1, 3> to_subtract = center_of_moving_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;
    Eigen::Matrix<double, 3, 3> covariance_of_moving_surface =
        points_in_matrix_form.transpose() * points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(
        covariance_of_moving_surface, Eigen::ComputeFullV | Eigen::ComputeFullU);
    assert(svd.computeU());
    auto rotation_of_moving_image = svd.matrixU();
    rotation_of_moving_image.col(2) =
        rotation_of_moving_image.col(0).cross(rotation_of_moving_image.col(1));
    pca_alignement.block<3, 3>(0, 0) = rotation_of_moving_image;
    pca_alignement.block<3, 1>(0, 3) = center_of_moving_image;
  
    Eigen::Matrix<double, 4, 4> inv_pca_alignement = pca_alignement;
    inv_pca_alignement.block<3, 3>(0, 0) = rotation_of_moving_image.transpose();
    inv_pca_alignement.block<3, 1>(0, 3) =
        -rotation_of_moving_image.transpose() * center_of_moving_image;
  
    pointIterator = mesh->GetPoints()->Begin();
  
    while (pointIterator != mesh->GetPoints()->End()) {
      auto &p = pointIterator->Value();
      Eigen::Matrix<double, 4, 1> original_point =
          Eigen::Matrix<double, 4, 1>::Ones();
      original_point[0] = p[0];
      original_point[1] = p[1];
      original_point[2] = p[2];
      Eigen::Matrix<double, 4, 1> transformed_point =
          inv_pca_alignement * original_point;
      p[0] = transformed_point[0];
      p[1] = transformed_point[1];
      p[2] = transformed_point[2];
      ++pointIterator;
    }
    return {pca_alignement, prune_surface(mesh)};
  }
  
  std::tuple<double, Eigen::Matrix<double, 4, 4>>
  icp_registration(Eigen::Matrix4d initial_config,
                   itk::PointSet<double, 3>::Pointer &fixed_point_cloud,
                   itk::PointSet<double, 3>::Pointer &moving_point_cloud) {
    auto metric =
        itk::EuclideanDistancePointMetric<itk::PointSet<double, 3>,
                                          itk::PointSet<double, 3>>::New();
    auto transform = itk::Euler3DTransform<double>::New();
    auto optimizer = itk::LevenbergMarquardtOptimizer::New();
    optimizer->SetUseCostFunctionGradient(false);
  
    auto registration = itk::PointSetToPointSetRegistrationMethod<
        itk::PointSet<double, 3>, itk::PointSet<double, 3>>::New();
    itk::LevenbergMarquardtOptimizer::ScalesType scales(
        transform->GetNumberOfParameters());
    constexpr double translationScale = 1000.0;
    constexpr double rotationScale = 1.0;
    scales[0] = 1.0 / rotationScale;
    scales[1] = 1.0 / rotationScale;
    scales[2] = 1.0 / rotationScale;
    scales[3] = 1.0 / translationScale;
    scales[4] = 1.0 / translationScale;
    scales[5] = 1.0 / translationScale;
  
    unsigned long numberOfIterations = 2000;
    double gradientTolerance = 1e-4;
    double valueTolerance = 1e-4;
    double epsilonFunction = 1e-5;
  
    optimizer->SetScales(scales);
    optimizer->SetNumberOfIterations(numberOfIterations);
    optimizer->SetValueTolerance(valueTolerance);
    optimizer->SetGradientTolerance(gradientTolerance);
    optimizer->SetEpsilonFunction(epsilonFunction);
  
    auto initialTransform = itk::Euler3DTransform<double>::New();
    itk::Euler3DTransform<double>::Pointer matrix =
        itk::Euler3DTransform<double>::New();
    itk::Vector<double, 3> origin;
    itk::Matrix<double> direction;
  
    for (size_t row = 0; row < 3; ++row) {
      origin[row] = initial_config(row, 3);
      for (size_t col = 0; col < 3; ++col)
        direction(row, col) = initial_config(row, col);
    }
  
    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);
  
    registration->SetInitialTransformParameters(
        initialTransform->GetParameters());
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
    registration->SetTransform(transform);
    registration->SetFixedPointSet(fixed_point_cloud);
    registration->SetMovingPointSet(moving_point_cloud);
  
    update_ikt_filter(registration);
  
    Eigen::Matrix<double, 4, 4> final_transformation =
        Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t row = 0; row < 3; ++row) {
      final_transformation(row, 3) = transform->GetOffset()[row];
      for (size_t col = 0; col < 3; ++col)
        final_transformation(row, col) = transform->GetMatrix()(row, col);
    }
  
    return {optimizer->GetValue().two_norm(), final_transformation};
  }
  
  template <typename pixel_type>
  int modify_image_with_transform(
      Eigen::Matrix<double, 4, 4> transform,
      typename itk::Image<pixel_type, 3>::Pointer image) {
    itk::Point<double, 3> origin;
    itk::Matrix<double> direction;
    for (size_t row = 0; row < 3; ++row) {
      origin[row] = transform(row, 3);
      for (size_t col = 0; col < 3; ++col)
        direction(row, col) = transform(row, col);
    }
    image->SetOrigin(origin);
    image->SetDirection(direction);
    return 1;
  };
  
  template <typename pixel_type>
  void print_image_with_transform(
      typename itk::Image<pixel_type, 3>::Pointer image,
      const std::string &image_path) {
    auto writer = itk::ImageFileWriter<typename itk::Image<pixel_type, 3>>::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    update_ikt_filter(writer);
    return;
  };
  
  template <typename PixelType> struct info_solve_registration {
    using ImageType = itk::Image<PixelType, 3>;
    typename ImageType::ConstPointer fixed_image;
    typename ImageType::ConstPointer moving_image;
    std::optional<itk::ImageMaskSpatialObject<3>::ConstPointer> fixed_image_mask;
    std::optional<itk::ImageMaskSpatialObject<3>::ConstPointer> moving_image_mask;
    const Eigen::Matrix<double, 4, 4> &initial_rotation;
  };
  
  constexpr size_t size_info = 3;
  
  struct RegistrationData {
    // internal data pre-allocated to save time
    using MaskType = itk::ImageMaskSpatialObject<3>;
    using ImageType = typename info_solve_registration<PixelType>::ImageType;
    using ImageRegistrationType =
        typename info_solve_registration<PixelType>::ImageType;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using InterpolatorType =
        itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType =
        itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                         ImageRegistrationType>;
    using RegistrationType =
        itk::ImageRegistrationMethodv4<ImageRegistrationType,
                                       ImageRegistrationType, TransformType>;
  
    RegistrationData(){};
  };
  
  struct RegistrationParameters {
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t, size_info> piramid_sizes;
    std::array<double, size_info> bluering_sizes;
  
    RegistrationParameters(size_t in_bin_numbers, double in_relative_scales,
                           double in_learning_rate, double in_sampling_percentage,
                           double in_relaxation_factor,
                           size_t in_convergence_window_size,
                           size_t in_optimization_iterations,
                           std::array<size_t, size_info> in_piramid_sizes,
                           std::array<double, size_info> in_bluering_sizes)
        : bin_numbers{in_bin_numbers}, relative_scales{in_relative_scales},
          learning_rate{in_learning_rate},
          sampling_percentage{in_sampling_percentage},
          relaxation_factor{in_relaxation_factor},
          convergence_window_size{in_convergence_window_size},
          optimization_iterations{in_optimization_iterations},
          piramid_sizes{in_piramid_sizes}, bluering_sizes{in_bluering_sizes} {}
  };
  
  template <typename TRegistration>
  class RegistrationInterfaceCommand : public itk::Command {
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
    void Execute(itk::Object *object, const itk::EventObject &event) override {
      if (!(itk::MultiResolutionIterationEvent().CheckEvent(&event))) {
        return;
      }
      auto registration = static_cast<RegistrationPointer>(object);
      auto optimizer =
          static_cast<OptimizerPointer>(registration->GetModifiableOptimizer());
      if (registration->GetCurrentLevel() == 0) {
        optimizer->SetLearningRate(0.1);
        optimizer->SetMinimumStepLength(0.1);
        optimizer->SetMaximumStepSizeInPhysicalUnits(0.2);
      } else {
        optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
        optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() * 0.2);
        optimizer->SetMaximumStepSizeInPhysicalUnits(
            optimizer->GetMaximumStepSizeInPhysicalUnits() * 0.2);
      }
    }
  
    void Execute(const itk::Object *, const itk::EventObject &) override {
      return;
    }
  };
  
  template <typename PixelType>
  std::tuple<double, Eigen::Matrix<double, 4, 4>, Eigen::Matrix<double, 4, 4>>
  solve_registration(const info_solve_registration<PixelType> &info_registration,
                     const RegistrationParameters &parameters) {
    using MaskType = itk::ImageMaskSpatialObject<3>;
    using ImageType = typename info_solve_registration<PixelType>::ImageType;
    using ImageRegistrationType =
        typename info_solve_registration<PixelType>::ImageType;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using InterpolatorType =
        itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType =
        itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                         ImageRegistrationType>;
    using RegistrationType =
        itk::ImageRegistrationMethodv4<ImageRegistrationType,
                                       ImageRegistrationType, TransformType>;
  
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    typename InterpolatorType::Pointer interpolator_moving =
        InterpolatorType::New();
    typename InterpolatorType::Pointer interpolator_fixed =
        InterpolatorType::New();
  
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
  
    metric->SetNumberOfHistogramBins(parameters.bin_numbers);
  
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
  
    if (info_registration.fixed_image_mask)
      metric->SetFixedImageMask(*info_registration.fixed_image_mask);
    if (info_registration.moving_image_mask)
      metric->SetMovingImageMask(*info_registration.moving_image_mask);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);
  
    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix =
        itk::Euler3DTransform<double>::New();
    itk::Vector<double, 3> origin;
    itk::Matrix<double> direction;
  
    for (size_t row = 0; row < 3; ++row) {
      origin[row] = info_registration.initial_rotation(row, 3);
      for (size_t col = 0; col < 3; ++col)
        direction(row, col) = info_registration.initial_rotation(row, col);
    }
  
    Eigen::Matrix<double, 4, 4> transformation =
        info_registration.initial_rotation;
  
    if (!(transformation.block<3, 3>(0, 0).transpose() *
          transformation.block<3, 3>(0, 0))
             .isDiagonal()) {
      std::cout << "failure to initialize rotation matrix...\n";
      std::cout << "values are: \n";
      std::cout << transformation.block<3, 3>(0, 0) << std::endl;
      std::cout << "the multiplication with itself is:";
      std::cout << transformation.block<3, 3>(0, 0).transpose() *
                       transformation.block<3, 3>(0, 0)
                << std::endl;
      throw std::runtime_error("failure to initialize the rotation matrix");
    }
    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);
  
    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);
  
    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());
  
    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = 1.0 / parameters.relative_scales;
    optimizerScales[4] = 1.0 / parameters.relative_scales;
    optimizerScales[5] = 1.0 / parameters.relative_scales;
  
    optimizer->SetScales(optimizerScales);
  
    optimizer->SetNumberOfIterations(parameters.optimization_iterations);
  
    optimizer->SetLearningRate(parameters.learning_rate);
    optimizer->SetMinimumStepLength(0.0001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{parameters.convergence_window_size};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(parameters.relaxation_factor);
  
    typename RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(size_info);
    typename RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(size_info);
  
    for (size_t i = 0; i < size_info; ++i) {
      shrinkFactorsPerLevel[i] = parameters.piramid_sizes[i];
      smoothingSigmasPerLevel[i] = parameters.bluering_sizes[i];
    }
  
    registration->SetNumberOfLevels(size_info);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);
  
    if (parameters.sampling_percentage > 0.99) {
      typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
          RegistrationType::MetricSamplingStrategyEnum::NONE;
      registration->SetMetricSamplingStrategy(samplingStrategy);
    } else {
      typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
          RegistrationType::MetricSamplingStrategyEnum::REGULAR;
      registration->MetricSamplingReinitializeSeed(std::rand());
      registration->SetMetricSamplingStrategy(samplingStrategy);
      registration->SetMetricSamplingPercentage(parameters.sampling_percentage);
    }
  
    registration->SetInPlace(false);
  
    using CommandType = RegistrationInterfaceCommand<RegistrationType>;
    auto command = CommandType::New();
    registration->AddObserver(itk::MultiResolutionIterationEvent(), command);
  
    try {
      registration->Update();
    } catch (const itk::ExceptionObject &err) {
      std::cout << "ExceptionObject caught !" << std::endl;
      std::cout << err << std::endl;
      return {100.0, Eigen::Matrix<double, 4, 4>::Identity(),
              info_registration.initial_rotation};
    }
    TransformType::Pointer final_registration =
        registration->GetModifiableTransform();
    Eigen::Matrix<double, 4, 4> final_transformation =
        Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t row = 0; row < 3; ++row) {
      final_transformation(row, 3) = final_registration->GetOffset()[row];
      for (size_t col = 0; col < 3; ++col)
        final_transformation(row, col) =
            final_registration->GetMatrix()(row, col);
    }
    return {optimizer->GetValue(), final_transformation,
            info_registration.initial_rotation};
  }
  
  void write_point_set(const std::string &filename,
                       itk::PointSet<double, 3>::Pointer pointset) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open file: " << filename << std::endl;
      return;
    }
    file << std::fixed << std::setprecision(6);
    for (itk::PointSet<double, 3>::PointsContainer::Iterator iterator =
             pointset->GetPoints()->Begin();
         iterator < pointset->GetPoints()->End(); ++iterator)
      file << iterator->Value()[0] << " " << iterator->Value()[1] << " "
           << iterator->Value()[2] << "\n";
    file.close();
  }
  
  template <typename PixelType>
  double evaluate_mi_with_both_images(
      const info_solve_registration<PixelType> &info_registration,
      size_t bin_numbers) {
    using RegistrationPixelType = PixelType;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image<PixelType, Dimension>;
    using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
    using MetricType =
        itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                         ImageRegistrationType>;
    using InterpolatorType =
        itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
    using TransformType = itk::VersorRigid3DTransform<double>;
  
    typename InterpolatorType::Pointer interpolator_moving =
        InterpolatorType::New();
    typename InterpolatorType::Pointer interpolator_fixed =
        InterpolatorType::New();
  
    auto metric = MetricType::New();
    metric->SetNumberOfHistogramBins(bin_numbers);
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    metric->SetFixedImage(info_registration.fixed_image);
    metric->SetFixedImageMask(*info_registration.fixed_image_mask);
    metric->SetMovingImageMask(*info_registration.moving_image_mask);
    metric->SetMovingImage(info_registration.moving_image);
    metric->SetFixedInterpolator(interpolator_fixed);
    metric->SetMovingInterpolator(interpolator_moving);
  
    auto initialTransform = TransformType::New();
    initialTransform->SetIdentity();
    metric->SetTransform(initialTransform);
  
    TransformType::ParametersType displacement(
        initialTransform->GetNumberOfParameters());
    displacement.Fill(0);
    try {
      metric->Initialize();
      metric->SetParameters(displacement);
  
      return metric->GetValue();
    } catch (const itk::ExceptionObject &err) {
      return 100.0;
    }
  }

  ImageType::Pointer scanned_volume = nullptr;
  ImageType::Pointer masked_moving_volume = nullptr;

Eigen::Matrix<double,4,4> main_solve_registration(ImageType::Pointer fixed_volume,ImageType::Pointer moving_volume){
  auto [point_cloud_fixed, mask_fixed_image] = extract_point_cloud(fixed_volume,ExtractionSurfaceInfo<true>{3, 0.95, "fixed", 5, 5});
  auto [transformation_acording_to_pca_fixed, tmp_fixed_point_set] = recentered_data(point_cloud_fixed);
  auto fixed_point_set = tmp_fixed_point_set;

  auto [point_cloud_moving, mask_moving_image] = extract_point_cloud(moving_volume,ExtractionSurfaceInfo<true>{3, 0.8, "moving", 5, 5});
  auto [transformation_acording_to_pca_moving, tmp_moving_point_set] = recentered_data(point_cloud_moving);
  auto moving_point_set = tmp_moving_point_set;

  std::vector<Eigen::Matrix<double, 4, 4>> initial_guesses_icp;
  for (double angle = 0; angle < 360.0; angle += 100.0) {
    initial_guesses_icp.push_back(transform_x(rad2deg(angle)));
    initial_guesses_icp.push_back(transform_flipped_principal_component(rad2deg(angle)));
  }

  std::vector<std::tuple<double, Eigen::Matrix<double, 4, 4>>> full_runs_inner;
  {
    std::mutex mut;
    auto pool = curan::utilities::ThreadPool::create(6, curan::utilities::TERMINATE_ALL_PENDING_TASKS);
    size_t counter = 0;
    for (const auto &initial_config : initial_guesses_icp) {
      pool->submit("solving icp", [&]() {
            auto solution = icp_registration(initial_config, fixed_point_set,
                                             moving_point_set);
            {
              std::lock_guard<std::mutex> g{mut};
              full_runs_inner.emplace_back(solution);
              ++counter;
              std::printf("%.2f %% %.3f\n",
                          (counter / (double)initial_guesses_icp.size()) * 100,
                          std::get<0>(solution));
            }
          });
    }
  }

  Eigen::Matrix<double, 4, 4> best_transformation_icp;
  auto min_element_iter =std::min_element(full_runs_inner.begin(), full_runs_inner.end(), [](const auto &a, const auto &b) { return std::get<0>(a) < std::get<0>(b);});

  if (min_element_iter != full_runs_inner.end()) {
    auto [minimum, best_icp_transform] = *min_element_iter;
    best_transformation_icp = best_icp_transform;
  }

  Eigen::Matrix<double, 4, 4> Timage_origin_fixed =
      Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Matrix<double, 4, 4> Timage_origin_moving =
      Eigen::Matrix<double, 4, 4>::Identity();
  for (size_t row = 0; row < 3; ++row) {
    Timage_origin_fixed(row, 3) = fixed_volume->GetOrigin()[row];
    Timage_origin_moving(row, 3) = moving_volume->GetOrigin()[row];
    for (size_t col = 0; col < 3; ++col) {
      Timage_origin_fixed(row, col) = fixed_volume->GetDirection()(row, col);
      Timage_origin_moving(row, col) = moving_volume->GetDirection()(row, col);
    }
  }

  auto converter = [](itk::Image<float, 3>::ConstPointer image) {
    auto rescaler =
        itk::RescaleIntensityImageFilter<itk::Image<float, 3>,
                                         itk::Image<float, 3>>::New();
    rescaler->SetInput(image);
    rescaler->SetOutputMinimum(0);
    rescaler->SetOutputMaximum(itk::NumericTraits<unsigned char>::max());
    update_ikt_filter(rescaler);
    itk::Image<float, 3>::Pointer output = rescaler->GetOutput();
    return output;
  };

  auto fixed = converter(fixed_volume);
  auto moving = converter(moving_volume);

  modify_image_with_transform<float>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed,fixed);
  modify_image_with_transform<float>(best_transformation_icp *transformation_acording_to_pca_moving.inverse() *Timage_origin_moving,moving);

  modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, mask_fixed_image);
  modify_image_with_transform<unsigned char>(best_transformation_icp *transformation_acording_to_pca_moving.inverse() *Timage_origin_moving,mask_moving_image);

  std::ofstream myfile{"results_of_fullscale_optimization.csv"};
  myfile << "run,iterations,bins,sampling percentage,relative_scales,learning "
            "rate,relaxation,convergence window,piramid sizes,bluring "
            "sizes,best cost,total time,true_error\n";

  // Optimizer parameters
  constexpr size_t local_permut = 1;

  std::vector<size_t> bin_numbers{100};
  std::vector<double> percentage_numbers{0.5};
  std::vector<double> relative_scales{1000.0};
  std::vector<double> learning_rate{.1};
  std::vector<double> relaxation_factor{0.8};
  std::vector<size_t> optimization_iterations{300};
  std::vector<size_t> convergence_window_size{40};

  std::array<std::array<size_t, size_info>, local_permut> piramid_sizes{
      {{3, 2, 1}}};
  std::array<std::array<double, size_info>, local_permut> bluering_sizes{
      {{2, 0, 0}}};

  size_t total_permutations = optimization_iterations.size() *
                              bin_numbers.size() * percentage_numbers.size() *
                              relative_scales.size() * learning_rate.size() *
                              relaxation_factor.size() *
                              convergence_window_size.size() *
                              piramid_sizes.size() * bluering_sizes.size();

  std::cout << "total permutations: " << total_permutations << std::endl;

  std::vector<std::tuple<double, Eigen::Matrix<double, 4, 4>>> full_runs;

  auto transformed_mask_fixed_image = itk::ImageMaskSpatialObject<3>::New();
  transformed_mask_fixed_image->SetImage(mask_fixed_image);
  update_ikt_filter(transformed_mask_fixed_image);
  auto transformed_mask_moving_image = itk::ImageMaskSpatialObject<3>::New();
  transformed_mask_moving_image->SetImage(mask_moving_image);
  update_ikt_filter(transformed_mask_moving_image);

  {
    std::mutex mut;
    auto pool = curan::utilities::ThreadPool::create(8, curan::utilities::TERMINATE_ALL_PENDING_TASKS);
    size_t total_runs = 0;
    for (const auto &bin_n : bin_numbers)
      for (const auto &percent_n : percentage_numbers)
        for (const auto &rel_scale : relative_scales)
          for (const auto &learn_rate : learning_rate)
            for (const auto &relax_factor : relaxation_factor)
              for (const auto &wind_size : convergence_window_size)
                for (const auto &pira_size : piramid_sizes)
                  for (const auto &iters : optimization_iterations)
                    for (const auto &blur_size : bluering_sizes) {

                      curan::utilities::Job job{
                          "solving registration", [&transformation_acording_to_pca_fixed,&best_transformation_icp,&transformation_acording_to_pca_moving,&total_permutations,&full_runs,&myfile,total_runs,&mut,fixed,moving,transformed_mask_fixed_image,transformed_mask_moving_image,bin_n,rel_scale,learn_rate,percent_n,relax_factor,wind_size,iters,pira_size,blur_size]() {
                            std::chrono::steady_clock::time_point begin =
                                std::chrono::steady_clock::now();
                            auto [cost, transformation, initial_transform] =
                                solve_registration(
                                    info_solve_registration<float>{
                                        fixed, moving,
                                        transformed_mask_fixed_image,
                                        transformed_mask_moving_image,
                                        Eigen::Matrix<double, 4,
                                                      4>::Identity()},
                                    RegistrationParameters{
                                        bin_n, rel_scale, learn_rate, percent_n,
                                        relax_factor, wind_size, iters,
                                        pira_size, blur_size});
                            std::chrono::steady_clock::time_point end =
                                std::chrono::steady_clock::now();
                            double duration =
                                std::chrono::duration_cast<
                                    std::chrono::seconds>(end - begin)
                                    .count();
                            {
                              std::lock_guard<std::mutex> g{mut};
                              myfile << total_runs << "," << iters << ","
                                     << bin_n << "," << percent_n << ","
                                     << rel_scale << "," << learn_rate << ","
                                     << relax_factor << "," << wind_size
                                     << ", {";
                              for (const auto &val : pira_size)
                                myfile << val << " ";
                              myfile << "}, {";
                              for (const auto &val : blur_size)
                                myfile << val << " ";
                              myfile
                                  << "}," << cost << "," << duration << ","
                                  << get_error(
                                         transformation_acording_to_pca_fixed *
                                         transformation.inverse() *
                                         best_transformation_icp *
                                         transformation_acording_to_pca_moving
                                             .inverse())
                                         .mean()
                                  << std::endl;

                              
                              full_runs.emplace_back(cost, transformation);
                              std::printf(
                                  "mi (%.2f %%)\n",
                                  (total_runs / (double)total_permutations) *
                                      100);
                            }
                          }};
                      ++total_runs;
                      pool->submit(job);
                    }
  }

  std::sort(full_runs.begin(), full_runs.end(),
            [](const std::tuple<double, Eigen::Matrix4d> &a,
               const std::tuple<double, Eigen::Matrix4d> &b) {
              return std::get<0>(a) < std::get<0>(b);
            });
  const double pi = std::atan(1) * 4;
  auto [cost, best_transformation_mi] = full_runs[0];

  std::cout << "\n error with MI:"
            << get_error(transformation_acording_to_pca_fixed *
                         best_transformation_mi.inverse() *
                         best_transformation_icp *
                         transformation_acording_to_pca_moving.inverse());
  std::cout << "\n error with ICP:"
            << get_error(transformation_acording_to_pca_fixed *
                         best_transformation_icp *
                         transformation_acording_to_pca_moving.inverse())
            << "\n\n\n";

  return transformation_acording_to_pca_fixed*best_transformation_mi.inverse()*best_transformation_icp*transformation_acording_to_pca_moving.inverse();
}


constexpr bool display_ultrasound_with_plus_homogeneous_data = false;

class RobotState
{
    std::atomic<bool> commit_senpuko = false;
    std::atomic<bool> f_record_frame = false;
    std::atomic<bool> f_regenerate_integrated_reconstructor_frame = false;
    std::atomic<bool> f_inject_frame = false;

public:
    vsg::ref_ptr<curan::renderable::Renderable> robot;
    curan::renderable::Window &window_pointer;
    curan::image::BoundingBox4Reconstruction box_class;
    vsg::ref_ptr<curan::renderable::Renderable> rendered_box;
    curan::image::IntegratedReconstructor::Info integrated_volume_create_info;
    vsg::ref_ptr<curan::renderable::Renderable> integrated_volume;
    std::atomic<curan::renderable::Renderable *> raw_dynamic_texture = nullptr;
    vsg::ref_ptr<curan::renderable::Renderable> dynamic_texture;
    vsg::dmat4 calibration_matrix;
    vsg::ref_ptr<curan::renderable::Renderable> moving_volume;

    RobotState(curan::renderable::Window &wind) : window_pointer{wind}, integrated_volume_create_info{{{0.1, 0.1, 0.1}}, {{0, 0, 0}}, {{10, 10, 10}}, {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}}
    {
    }

    RobotState(const RobotState &) = delete;
    RobotState(RobotState &&) = delete;
    RobotState &operator=(const RobotState &) = delete;
    RobotState &operator=(RobotState &&) = delete;

    operator bool()
    {
        return commit_senpuko.load();
    }

    void operator()(bool value)
    {
        commit_senpuko.store(value);
    }

    enum RecordStatus
    {
        START_RECORDING_FOR_BOX_UPDATE,
        NOT_RECORDING
    };

    void record_frames(RecordStatus status)
    {
        f_record_frame.store(status == START_RECORDING_FOR_BOX_UPDATE);
    }

    bool record_frames()
    {
        return f_record_frame.load();
    }

    enum GenerateStatus
    {
        GENERATE_VOLUME,
        ALREADY_GENERATED
    };

    void generate_volume(GenerateStatus status)
    {
        // std::cout << "flag \"generate_volume\" updated!!!";
        f_regenerate_integrated_reconstructor_frame.store(status == GENERATE_VOLUME);
    }

    bool generate_volume()
    {
        return f_regenerate_integrated_reconstructor_frame.load();
    }

    enum InjectVolumeStatus
    {
        INJECT_FRAME,
        FREEZE_VOLUME
    };

    void inject_frame(InjectVolumeStatus status)
    {
        // std::cout << "flag \"inject_frame\" updated!!!";
        f_inject_frame.store(status == INJECT_FRAME);
    }

    bool inject_frame()
    {
        return f_inject_frame.load();
    }
};

enum WindowSpecification
{
    ROI_SPECIFICATION,
    VOLUME_RECONSTRUCTION,
    REGISTRATION
};

struct ApplicationState
{
    WindowSpecification specification{ROI_SPECIFICATION};
    bool operation_in_progress = false;
    bool show_error = false;
    bool show_sucess = false;
    std::mutex mut;
    std::string operation_description;
    std::string success_description;
    ImVec2 padding{0, 40};
    std::shared_ptr<curan::utilities::ThreadPool> pool;
    RobotState robot_state;
    std::string filename{CURAN_COPIED_RESOURCE_PATH "/reconstruction_results.mha"};
    ImageType::Pointer scanned_volume = nullptr;
    ImageType::Pointer masked_moving_volume = nullptr;
    ImageType::Pointer original_moving_volume = nullptr;

    ApplicationState(curan::renderable::Window &wind) : robot_state{wind}
    {
        /*
        We need three threads:
        1) one for volumetric reconstruction
        2) another for the communication thread
        3) Another to process the UI tasks in sequence
        */
        pool = curan::utilities::ThreadPool::create(3);
    }

    void showMainWindow()
    {
        static bool first_time = true;
        if(first_time){
            ImGui::SetNextWindowSize(ImVec2{400,400},ImGuiCond_Always);
            first_time = false;
        }
        
        ImGui::Begin("Volume Reconstruction", NULL, ImGuiWindowFlags_MenuBar);
        if (ImGui::BeginMenuBar())
        {
            if (ImGui::BeginMenu("Mode"))
            {
                if (ImGui::MenuItem("ROI specification", "Ctrl+O"))
                {
                    if (operation_in_progress && specification != ROI_SPECIFICATION)
                    {
                        std::lock_guard<std::mutex> g{mut};
                        show_error = true;
                    }
                    else
                        specification = ROI_SPECIFICATION;
                }
                if (ImGui::MenuItem("Reconstruction", "Ctrl+S"))
                {
                    if (operation_in_progress && specification != VOLUME_RECONSTRUCTION)
                    {
                        std::lock_guard<std::mutex> g{mut};
                        show_error = true;
                    }
                    else
                        specification = VOLUME_RECONSTRUCTION;
                }
                if (ImGui::MenuItem("Registration", "Ctrl+F"))
                {
                    if (operation_in_progress && specification != REGISTRATION)
                    {
                        std::lock_guard<std::mutex> g{mut};
                        show_error = true;
                    }
                    else
                        specification = REGISTRATION;
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        ImGui::Dummy(padding);
        ImGui::SameLine();
        bool local_copy;
        {
            std::lock_guard<std::mutex> g{mut};
            local_copy = operation_in_progress;
        }
        if (local_copy)
            ImGui::ProgressBar(-1.0f * ImGui::GetTime());


        switch (specification)
        {
        case ROI_SPECIFICATION:
            ImGui::Dummy(padding);
            ImGui::SameLine();
            ImGui::TextWrapped("Please start selecting your region of interest. Once the robot is in place, press the storing button."); // Display some text (you can use a format strings too)
            ImGui::Dummy(padding);
            ImGui::SameLine();
            showRegionOfInterestWindow();
            break;
        case VOLUME_RECONSTRUCTION:
            ImGui::Dummy(padding);
            ImGui::SameLine();
            ImGui::TextWrapped("Upon termination of your scanning routine, click the save button to store the volume for registration purpouses."); // Display some text (you can use a format strings too)
            ImGui::Dummy(padding);
            ImGui::SameLine();
            showReconstructionWindow();
            break;
        default:
            ImGui::Dummy(padding);
            ImGui::SameLine();
            ImGui::TextWrapped("Registration between pre-operative volume and scanned volume."); // Display some text (you can use a format strings too)
            ImGui::Dummy(padding);
            ImGui::SameLine();
            showRegistrationWindow();
            break;
        }
        if (ImGui::Button("Center Camera"))
        {

        }
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
        showOverlayErrorWindow();
        showOverlaySuccessWindow();
    }

    void showRegistrationWindow()
    {
        ImGui::Dummy(padding);
        if (ImGui::Button("Start Registration"))
        {
            {
                std::lock_guard<std::mutex> g{mut};
                operation_in_progress = true;
                operation_description = "saving volumetric reconstruction";
                robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
            }
            pool->submit("solve registration", [this](){                         
                auto transform_moving_to_fixed = main_solve_registration(masked_moving_volume,scanned_volume);
                {
                    std::lock_guard<std::mutex> g{mut};
                    success_description = "solved registration problem";
                    operation_in_progress = false;
                    show_sucess = true;
                    robot_state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
                }
             });
        }
    };

    void showReconstructionWindow()
    {
        ImGui::Dummy(padding);
        ImGui::SameLine();
        {
            std::lock_guard<std::mutex> g{mut};
            ImGui::InputText("Filename", &filename);
            filename = CURAN_COPIED_RESOURCE_PATH "/reconstruction_results.mha";
        }

        if (ImGui::Button("Save Volume"))
        {
            {
                std::lock_guard<std::mutex> g{mut};
                operation_in_progress = true;
                operation_description = "saving volumetric reconstruction";
                robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
            }
            pool->submit(curan::utilities::Job{"testing", [this]()
                                               {
                                                   nlohmann::json specified_box;
                                                   std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/specified_box.json");
                                                   in >> specified_box;

                                                   std::string timestamp_box = specified_box["timestamp"];

                                                   std::stringstream spacing_box_info;
                                                   std::string spacing_box = specified_box["spacing"];
                                                   spacing_box_info << spacing_box;
                                                   Eigen::MatrixXd spacing = curan::utilities::convert_matrix(spacing_box_info, ',');
                                                   std::string origin_box = specified_box["origin"];
                                                   std::stringstream origin_box_info;
                                                   origin_box_info << origin_box;
                                                   Eigen::MatrixXd origin = curan::utilities::convert_matrix(origin_box_info, ',');
                                                   std::string size_box = specified_box["size"];
                                                   std::stringstream size_box_info;
                                                   size_box_info << size_box;

                                                   Eigen::MatrixXd size = curan::utilities::convert_matrix(size_box_info, ',');
                                                   std::string direction_box = specified_box["direction"];
                                                   std::stringstream direction_box_info;
                                                   direction_box_info << direction_box;

                                                   Eigen::MatrixXd direction = curan::utilities::convert_matrix(direction_box_info, ',');

                                                   if (spacing.rows() != 1 || spacing.cols() != 3)
                                                       throw std::runtime_error("The supplied spacing has an incorrect dimension (expected : [1x3])");

                                                   if (origin.rows() != 1 || origin.cols() != 3)
                                                       throw std::runtime_error("The supplied origin has an incorrect dimension (expected : [1x3])");

                                                   if (size.rows() != 1 || size.cols() != 3)
                                                       throw std::runtime_error("The supplied size has an incorrect dimension (expected : [1x3])");

                                                   if (direction.rows() != 3 || direction.cols() != 3)
                                                       throw std::runtime_error("The supplied direction has an incorrect dimension (expected : [3x3])");

                                                   itk::Size<3U> output_size;
                                                   output_size = robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_output_size();

                                                   using PixelType = float;
 
                                                   using ImportFilterType = itk::ImportImageFilter<PixelType, 3>;
                                                   auto importFilter = ImportFilterType::New();
                                                   ImportFilterType::IndexType start;
                                                   start.Fill(0);

                                                   ImportFilterType::RegionType region;
                                                   region.SetIndex(start);
                                                   region.SetSize(output_size);

                                                   importFilter->SetRegion(region);

                                                   const itk::SpacePrecisionType output_origin[3] = {origin(0, 0) * 1000, origin(0, 1) * 1000, origin(0, 2) * 1000};
                                                   importFilter->SetOrigin(output_origin);

                                                   const itk::SpacePrecisionType output_spacing[3] = {spacing(0, 0) * 1000, spacing(0, 1) * 1000, spacing(0, 2) * 1000};
                                                   importFilter->SetSpacing(output_spacing);
                                                   const unsigned int numberOfPixels = output_size[0] * output_size[1] * output_size[2];

                                                   itk::Matrix<double> orientation;
                                                   for (size_t row = 0; row < 3; ++row)
                                                       for (size_t col = 0; col < 3; ++col)
                                                           orientation(row, col) = direction(row, col);
                                                   importFilter->SetDirection(orientation);

                                                   const bool importImageFilterWillOwnTheBuffer = false;
                                                   if (robot_state.integrated_volume.get() == nullptr)
                                                       return;
                                                   float *my_beatiful_pointer = robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_texture_data()->data();
                                                   importFilter->SetImportPointer(my_beatiful_pointer, numberOfPixels, importImageFilterWillOwnTheBuffer);

                                                   using WriterType = itk::ImageFileWriter<ImageType>;
                                                   WriterType::Pointer writer = WriterType::New();

                                                   {
                                                       std::lock_guard<std::mutex> g{mut};
                                                       writer->SetFileName(filename);
                                                   }
                                                   
                                                   using RescaleType = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>;
                                                   auto rescale = RescaleType::New();
                                                   rescale->SetInput(importFilter->GetOutput());
                                                   rescale->SetOutputMinimum(0.0);
                                                   rescale->SetOutputMaximum(1.0);
                                                   writer->SetInput(rescale->GetOutput());
                                                   try{
                                                    writer->Update();
                                                    scanned_volume = DeepCopy(rescale->GetOutput());
                                                   } catch(...){
                                                    std::lock_guard<std::mutex> g{mut};
                                                    success_description = "failed to store/record the volume";
                                                    operation_in_progress = false;
                                                    show_error = true;
                                                    robot_state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
                                                    return;
                                                   }
                                                
                                                   {
                                                       std::lock_guard<std::mutex> g{mut};
                                                       success_description = "saved volume information";
                                                       operation_in_progress = false;
                                                       show_sucess = true;
                                                       robot_state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
                                                   }
                                               }});
        }
    };

    void showRegionOfInterestWindow()
    {
        static bool local_record_data = false;
        static bool previous_local_record_data = local_record_data;
        ImGui::Checkbox("Start Frame Collection", &local_record_data);
        if (local_record_data)
        {
            std::lock_guard<std::mutex> g{mut};
            operation_in_progress = true;
            operation_description = "collecting data from ultrasound for region of interest";
            robot_state.record_frames(RobotState::START_RECORDING_FOR_BOX_UPDATE);
            robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
            if (previous_local_record_data != local_record_data)
            {
                robot_state.box_class.reset();
            }
        }
        else
            robot_state.record_frames(RobotState::NOT_RECORDING);
        previous_local_record_data = local_record_data;
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Save Region of Interest"))
        {
            robot_state.record_frames(RobotState::NOT_RECORDING);
            local_record_data = false;
            operation_in_progress = true;
            operation_description = "saving region of interest";

            pool->submit(curan::utilities::Job{"saving region of interest", [this]()
                                               {
                                                   auto final_box = robot_state.box_class.get_final_volume_vertices();
                                                   vsg::dmat3 rotation_0_1;
                                                   Eigen::Matrix<double, 3, 3> final_box_orientation;
                                                   for (size_t col = 0; col < 3; ++col)
                                                       for (size_t row = 0; row < 3; ++row)
                                                       {
                                                           rotation_0_1(col, row) = final_box.axis[col][row];
                                                           final_box_orientation(row, col) = final_box.axis[col][row];
                                                       }

                                                   vsg::dvec3 position_of_center_in_global_frame;
                                                   position_of_center_in_global_frame[0] = final_box.center[0];
                                                   position_of_center_in_global_frame[1] = final_box.center[1];
                                                   position_of_center_in_global_frame[2] = final_box.center[2];

                                                   vsg::dvec3 position_in_local_box_frame;
                                                   position_in_local_box_frame[0] = final_box.extent[0];
                                                   position_in_local_box_frame[1] = final_box.extent[1];
                                                   position_in_local_box_frame[2] = final_box.extent[2];

                                                   auto global_corner_position = position_of_center_in_global_frame - rotation_0_1 * position_in_local_box_frame;
                                                   nlohmann::json specified_box;
                                                   specified_box["timestamp"] = curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::now());

                                                   constexpr size_t maximum_float_size = 62.5e6 * 0.5;
                                                   double new_spacing = std::cbrt((2 * final_box.extent[0] * 2 * final_box.extent[1] * 2 * final_box.extent[2]) / (maximum_float_size));

                                                   std::stringstream ss;
                                                   ss << new_spacing << " , " << new_spacing << " , " << new_spacing;
                                                   specified_box["spacing"] = ss.str();
                                                   ss.str("");
                                                   ss << global_corner_position[0] << " , " << global_corner_position[1] << " , " << global_corner_position[2];
                                                   specified_box["origin"] = ss.str();
                                                   ss.str("");
                                                   ss << 2 * final_box.extent[0] << " , " << 2 * final_box.extent[1] << " , " << 2 * final_box.extent[2];
                                                   specified_box["size"] = ss.str();
                                                   ss.str("");
                                                   Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");
                                                   ss << final_box_orientation.format(CleanFmt);
                                                   specified_box["direction"] = ss.str();

                                                   // write prettified JSON to another file
                                                   std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/specified_box.json");
                                                   o << specified_box;

                                                   std::string timestamp_box = specified_box["timestamp"];

                                                   std::stringstream spacing_box_info;
                                                   std::string spacing_box = specified_box["spacing"];
                                                   spacing_box_info << spacing_box;
                                                   Eigen::MatrixXd spacing = curan::utilities::convert_matrix(spacing_box_info, ',');
                                                   std::string origin_box = specified_box["origin"];
                                                   std::stringstream origin_box_info;
                                                   origin_box_info << origin_box;
                                                   Eigen::MatrixXd origin = curan::utilities::convert_matrix(origin_box_info, ',');
                                                   std::string size_box = specified_box["size"];
                                                   std::stringstream size_box_info;
                                                   size_box_info << size_box;

                                                   Eigen::MatrixXd size = curan::utilities::convert_matrix(size_box_info, ',');
                                                   std::string direction_box = specified_box["direction"];
                                                   std::stringstream direction_box_info;
                                                   direction_box_info << direction_box;

                                                   Eigen::MatrixXd direction = curan::utilities::convert_matrix(direction_box_info, ',');

                                                   if (spacing.rows() != 1 || spacing.cols() != 3)
                                                       throw std::runtime_error("The supplied spacing has an incorrect dimension (expected : [1x3])");

                                                   if (origin.rows() != 1 || origin.cols() != 3)
                                                       throw std::runtime_error("The supplied origin has an incorrect dimension (expected : [1x3])");

                                                   if (size.rows() != 1 || size.cols() != 3)
                                                       throw std::runtime_error("The supplied size has an incorrect dimension (expected : [1x3])");

                                                   if (direction.rows() != 3 || direction.cols() != 3)
                                                       throw std::runtime_error("The supplied direction has an incorrect dimension (expected : [3x3])");

                                                   std::array<double, 3> vol_origin = {origin(0, 0), origin(0, 1), origin(0, 2)};
                                                   std::array<double, 3> vol_spacing = {spacing(0, 0), spacing(0, 1), spacing(0, 2)};
                                                   std::array<double, 3> vol_size = {size(0, 0), size(0, 1), size(0, 2)};
                                                   std::array<std::array<double, 3>, 3> vol_direction;
                                                   vol_direction[0] = {direction(0, 0), direction(1, 0), direction(2, 0)};
                                                   vol_direction[1] = {direction(0, 1), direction(1, 1), direction(2, 1)};
                                                   vol_direction[2] = {direction(0, 2), direction(1, 2), direction(2, 2)};
                                                   curan::image::IntegratedReconstructor::Info recon_info{vol_spacing, vol_origin, vol_size, vol_direction};
                                                   recon_info.identifier = "reconstructor";

                                                   robot_state.integrated_volume_create_info = recon_info;
                                                   robot_state.generate_volume(RobotState::GenerateStatus::GENERATE_VOLUME);
                                                   robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
                                                   std::lock_guard<std::mutex> g{mut};
                                                   success_description = "saved volume information";
                                                   operation_in_progress = false;
                                                   show_sucess = true;
                                               }});
        }
    };

    void showOverlayErrorWindow()
    {
        if (!show_error)
            return;
        ImGui::Begin("Error Reporting", NULL, ImGuiWindowFlags_MenuBar);
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "Operation already in progress...");
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "%s", operation_description.data());
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Ok!"))
        {
            show_error = false;
        }
        ImGui::End();
    };

    void showOverlaySuccessWindow()
    {
        if (!show_sucess)
            return;
        ImGui::Begin("Operation Sucessefull", NULL, ImGuiWindowFlags_MenuBar);
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Operation finished");
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "%s", success_description.data());
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Ok!"))
        {
            show_sucess = false;
        }
        ImGui::End();
    };
};

bool process_image_message(RobotState &state, igtl::MessageBase::Pointer val)
{
    igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
    message_body->Copy(val);
    int c = message_body->Unpack(1);
    if (!(c & igtl::MessageHeader::UNPACK_BODY))
        return false; // failed to unpack message, therefore returning without doing anything
    if (state.dynamic_texture.get() == nullptr)
    {
        std::cout << "creating dynamic texture\n";
        int x, y, z;
        message_body->GetDimensions(x, y, z);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = y;
        infotexture.width = x;
        infotexture.spacing = {0.00018867924, 0.00018867924, 0.00018867924};
        infotexture.origin = {0.0, 0.0, 0.0};
        infotexture.identifier = "ultrasound";
        infotexture.builder = vsg::Builder::create();
        state.dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
        state.window_pointer << state.dynamic_texture;
        state.raw_dynamic_texture = state.dynamic_texture.get();

        std::cout << "creating bounding box texture\n";
        curan::renderable::Box::Info infobox;
        infobox.builder = vsg::Builder::create();
        infobox.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
        infobox.geomInfo.dx = vsg::vec3(1.0f, 0.0, 0.0);
        infobox.geomInfo.dy = vsg::vec3(0.0, 1.0f, 0.0);
        infobox.geomInfo.dz = vsg::vec3(0.0, 0.0, 1.0f);
        infobox.identifier = "roi";
        infobox.stateInfo.wireframe = true;
        infobox.geomInfo.position = vsg::vec3(0.5, 0.5, 0.5);
        state.rendered_box = curan::renderable::Box::make(infobox);
        state.window_pointer << state.rendered_box;
        std::cout << "creating box and ultrasound\n";
    }

    auto updateBaseTexture = [message_body](vsg::vec4Array2D &image)
    {
        try
        {
            int x, y, z;
            message_body->GetDimensions(x, y, z);
            unsigned char *scaller_buffer = (unsigned char *)message_body->GetScalarPointer();

            for (size_t r = 0; r < image.height(); ++r)
            {
                using value_type = typename vsg::vec4Array2D::value_type;
                value_type *ptr = &image.at(0, r);
                for (size_t c = 0; c < image.width(); ++c)
                {
                    auto val = *scaller_buffer / 255.0;
                    ptr->r = val;
                    ptr->g = val;
                    ptr->b = val;
                    ptr->a = 1.0f;
                    ++ptr;
                    ++scaller_buffer;
                }
            }
        }
        catch (std::exception &e)
        {
            std::cout << "exception : " << e.what() << std::endl;
        }
    };

    if (state.dynamic_texture.get() != nullptr)
        state.dynamic_texture.cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);

    igtl::Matrix4x4 image_transform;
    message_body->GetMatrix(image_transform);
    vsg::dmat4 homogeneous_transformation;
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            homogeneous_transformation(col, row) = image_transform[row][col];

    homogeneous_transformation(3, 0) *= 1e-3;
    homogeneous_transformation(3, 1) *= 1e-3;
    homogeneous_transformation(3, 2) *= 1e-3;

    auto product = homogeneous_transformation * state.calibration_matrix;

    if constexpr (display_ultrasound_with_plus_homogeneous_data)
    {
        if (state.dynamic_texture.get() != nullptr)
            state.dynamic_texture.cast<curan::renderable::DynamicTexture>()->update_transform(product);
    }

    OutputImageType::Pointer image_to_render;
    curan::image::igtl2ITK_im_convert(message_body, image_to_render);
    const itk::SpacePrecisionType spacing[3] = {0.00018867924, 0.00018867924, 0.00018867924};
    image_to_render->SetSpacing(spacing);

    itk::Matrix<double, 3, 3> itk_matrix;
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            itk_matrix(row, col) = product(col, row);

    image_to_render->SetDirection(itk_matrix);
    auto origin = image_to_render->GetOrigin();
    origin[0] = product(3, 0);
    origin[1] = product(3, 1);
    origin[2] = product(3, 2);
    image_to_render->SetOrigin(origin);

    static size_t counter = 0;
    constexpr size_t update_rate = 2;
    ++counter;

    if (state.record_frames() && (counter % update_rate == 0))
    {
        state.box_class.add_frame(image_to_render);
        state.box_class.update();
    }

    auto caixa = state.box_class.get_final_volume_vertices();
    vsg::dmat4 transform_matrix;

    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            transform_matrix(col, row) = image_to_render->GetDirection()[row][col];

    transform_matrix(3, 0) = image_to_render->GetOrigin()[0];
    transform_matrix(3, 1) = image_to_render->GetOrigin()[1];
    transform_matrix(3, 2) = image_to_render->GetOrigin()[2];

    vsg::dmat3 rotation_0_1;

    vsg::dmat4 box_transform_matrix = vsg::translate(0.0, 0.0, 0.0);

    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
        {
            box_transform_matrix(col, row) = caixa.axis[col][row];
            rotation_0_1(col, row) = box_transform_matrix(col, row);
        }

    vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = caixa.center[0];
    position_of_center_in_global_frame[1] = caixa.center[1];
    position_of_center_in_global_frame[2] = caixa.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = caixa.extent[0];
    position_in_local_box_frame[1] = caixa.extent[1];
    position_in_local_box_frame[2] = caixa.extent[2];

    auto global_corner_position = position_of_center_in_global_frame - rotation_0_1 * position_in_local_box_frame;

    box_transform_matrix(3, 0) = global_corner_position[0];
    box_transform_matrix(3, 1) = global_corner_position[1];
    box_transform_matrix(3, 2) = global_corner_position[2];
    box_transform_matrix(3, 3) = 1;

    state.rendered_box->cast<curan::renderable::Box>()->set_scale(caixa.extent[0] * 2, caixa.extent[1] * 2, caixa.extent[2] * 2);
    state.rendered_box->update_transform(box_transform_matrix);

    if (state.generate_volume())
    {
        state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
        state.window_pointer.erase("reconstructor");

        int clip_origin_x = (int)0;
        int clip_origin_y = (int)0;
        int x, y, z;
        message_body->GetDimensions(x, y, z);

        state.integrated_volume = curan::image::IntegratedReconstructor::make(state.integrated_volume_create_info);
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE).set_interpolation(curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION);
        state.window_pointer << state.integrated_volume;

        curan::image::Clipping desired_clip;
        desired_clip.clipRectangleOrigin[0] = clip_origin_x;
        desired_clip.clipRectangleOrigin[1] = clip_origin_y;
        desired_clip.clipRectangleSize[0] = x;
        desired_clip.clipRectangleSize[1] = y - 15;
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_clipping(desired_clip);

        state.generate_volume(RobotState::GenerateStatus::ALREADY_GENERATED);
        state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
    }

    if (state.integrated_volume.get() != nullptr && state.inject_frame())
    {
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_frame(image_to_render);
    }
    return true;
}

std::map<std::string, std::function<bool(RobotState &state, igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
    {"IMAGE", process_image_message}};

bool process_message(RobotState &state, size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
    assert(val.IsNotNull());
    if (er)
        return true;
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(state, val);
    return false;
}

//std::ofstream &get_file_handle()
//{
//    static bool initializing = true;
//    static std::string pathname = std::string{"joint_recording_"} + curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::now()) + std::string{".txt"};
//    static std::ofstream out{pathname};
//    if (initializing)
//    {
//        if (!out.is_open())
//            std::cout << "failed to create file with name:" << pathname << std::endl;
//        else
//            std::cout << "printing to file" << pathname << std::endl;
//    }
//    initializing = false;
//    return out;
//}

bool process_joint_message(RobotState &state, const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    static curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);

    static curan::robotic::State internal_state;
    internal_state.sampleTime = sample_time.count();
    double time = 0;

    if (er)
        return true;
    for (size_t joint_index = 0; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
        state.robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, message->angles[joint_index]);

    internal_state.q = message->angles;

    robot_model.update(internal_state);
    auto transform = robot_model.homogenenous_transformation();
    Eigen::Matrix<double,4,4> transform_imposed_by_plus_because_of_obscure_reaons = Eigen::Matrix<double,4,4>::Identity();
    transform_imposed_by_plus_because_of_obscure_reaons(0,3) = 0.1925;
    transform_imposed_by_plus_because_of_obscure_reaons(1,3) = 0.2125;
    transform_imposed_by_plus_because_of_obscure_reaons(2,3) = 0;

    transform = (transform*transform_imposed_by_plus_because_of_obscure_reaons).eval();

    vsg::dmat4 homogeneous_transformation;
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            homogeneous_transformation(col, row) = transform(row, col);

    auto product = homogeneous_transformation * state.calibration_matrix;
    if constexpr (!display_ultrasound_with_plus_homogeneous_data){
        if (state.raw_dynamic_texture != nullptr){
            auto local = state.raw_dynamic_texture.load();
            local->cast<curan::renderable::DynamicTexture>()->update_transform(product);
        }
    }
    return false;
}

int communication(RobotState &state, asio::io_context &context)
{
    asio::ip::tcp::resolver resolver(context);
    auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(context, resolver.resolve("localhost", std::to_string(18944)));

    auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
    {
        try
        {
            if (process_message(state, protocol_defined_val, er, val) || state)
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    client->connect(lam);
    std::cout << "connecting to client\n";

    asio::ip::tcp::resolver fri_resolver(context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(context, fri_resolver.resolve("172.31.1.148", std::to_string(50010)));

    auto lam_fri = [&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
    {
        try
        {
            if (process_joint_message(state, protocol_defined_val, er, message))
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    fri_client->connect(lam_fri);

    context.run();
    if(!state.window_pointer.erase("ultrasound"))
        std::cout << "could not erase ultrasound from image\n";
    std::cout << "stopped connecting to client\n";
    return 0;
}

void interface(vsg::CommandBuffer &cb, ApplicationState **app)
{
    if (*app != nullptr)
        (**app).showMainWindow();
}

int main(int argc, char **argv)
{
    asio::io_context context;
    ApplicationState *app_pointer = nullptr;

    curan::renderable::ImGUIInterface::Info info_gui{[pointer_to_address = &app_pointer](vsg::CommandBuffer &cb)
                                                     { interface(cb, pointer_to_address); }};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "Curan: Volume Reconstructor";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{2000, 1800};
    info.window_size = size;
    curan::renderable::Window window{info};

    ApplicationState application_state{window};
    app_pointer = &application_state;

    auto homogenenous_transform = Eigen::Matrix<double,4,4>::Identity();

    curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};
    //std::cout << "using calibration matrix: \n" << calibration.homogeneous_transformation() << std::endl;
    for (Eigen::Index row = 0; row < 4; ++row)
        for (Eigen::Index col = 0; col < 4; ++col)
            application_state.robot_state.calibration_matrix(col, row) = homogenenous_transform(row, col);

    curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json"};
    {
        auto image_reader_moving = itk::ImageFileReader<itk::Image<float, 3>>::New();
        image_reader_moving->SetFileName(trajectory_data.path_to_masked_image());
        using RescaleType = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>;
        auto rescale = RescaleType::New();
        rescale->SetInput(image_reader_moving->GetOutput());
        rescale->SetOutputMinimum(0.0);
        rescale->SetOutputMaximum(1.0);
        update_ikt_filter(rescale);
        application_state.masked_moving_volume = rescale->GetOutput();
    }
    {
        auto image_reader_moving = itk::ImageFileReader<itk::Image<float, 3>>::New();
        image_reader_moving->SetFileName(trajectory_data.path_to_original_image());
        update_ikt_filter(image_reader_moving);
        application_state.original_moving_volume = image_reader_moving->GetOutput();
    }

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    application_state.robot_state.robot = curan::renderable::SequencialLinks::make(create_info);
    window << application_state.robot_state.robot;

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = application_state.original_moving_volume->GetLargestPossibleRegion().GetSize()[0]; 
    volumeinfo.height = application_state.original_moving_volume->GetLargestPossibleRegion().GetSize()[1];
    volumeinfo.depth = application_state.original_moving_volume->GetLargestPossibleRegion().GetSize()[2];
    volumeinfo.spacing_x = application_state.original_moving_volume->GetSpacing()[0];
    volumeinfo.spacing_y = application_state.original_moving_volume->GetSpacing()[1];
    volumeinfo.spacing_z = application_state.original_moving_volume->GetSpacing()[2];

    auto volume = curan::renderable::Volume::make(volumeinfo);
    window << volume;

    Eigen::Matrix<double,4,4> transformation_to_sensor_base = Eigen::Matrix<double,4,4>::Identity();
    transformation_to_sensor_base(0,3) = application_state.original_moving_volume->GetOrigin()[0]*1e-3;
    transformation_to_sensor_base(1,3) = application_state.original_moving_volume->GetOrigin()[1]*1e-3;
    transformation_to_sensor_base(2,3) = application_state.original_moving_volume->GetOrigin()[2]*1e-3;

    auto direction = application_state.original_moving_volume->GetDirection();
    for(size_t r = 0; r < 3; ++r)
        for(size_t c = 0; c < 3; ++c)
            transformation_to_sensor_base(r,c) = direction(r,c);

    auto vsg_transformation_to_sensor_base = vsg::translate(0.0, 0.0, 0.0);
    for(size_t r = 0; r < 4; ++r)
        for(size_t c = 0; c < 4; ++c) 
            vsg_transformation_to_sensor_base(c, r) = transformation_to_sensor_base(r, c);
    volume->update_transform(vsg_transformation_to_sensor_base);
    volume->cast<curan::renderable::Volume>()->update_volume([moving_volume = application_state.original_moving_volume](vsg::floatArray3D &image) {updateBaseTexture3D<itk::Image<float, 3>>(image,moving_volume);});

    application_state.robot_state(true);

    application_state.pool->submit("communication with robot", [&](){ communication(application_state.robot_state, context); });
    application_state.pool->submit("reconstruct volume", [&](){
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(8);
        while (application_state.robot_state)
            if (application_state.robot_state.inject_frame() && application_state.robot_state.integrated_volume.get() != nullptr)
                                                                     application_state.robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
    });

    window.run();
    application_state.robot_state(false);
    return 0;
}
