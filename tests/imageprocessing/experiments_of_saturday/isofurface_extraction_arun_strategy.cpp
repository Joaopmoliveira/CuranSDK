#include <iostream>
#include <string>

#include "itkMeshSpatialObject.h"
#include "itkSpatialObjectReader.h"
#include "itkAutomaticTopologyMeshSource.h"
#include "imageprocessing/VolumetricRegistration.h"
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

inline double rad2deg(double in)
{
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians * in;
}

// Lamda to get a rotation matrix from a given angle
auto transform_x = [](double alpha)
{
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << 1.0, 0.0, 0.0,
        0.0, std::cos(alpha), -std::sin(alpha),
        0.0, std::sin(alpha), std::cos(alpha);
    Eigen::Matrix<double, 4, 4> T_rotation_extra = Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
};

auto transform_y = [](double alpha)
{
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << std::cos(alpha), 0.0, std::sin(alpha),
        0.0, 1.0, 0.0,
        -std::sin(alpha), 0.0, std::cos(alpha);
    Eigen::Matrix<double, 4, 4> T_rotation_extra = Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
};

auto transform_z = [](double alpha)
{
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << std::cos(alpha), -std::sin(alpha), 0.0,
        std::sin(alpha), std::cos(alpha), 0.0,
        0.0, 0.0, 1.0;
    Eigen::Matrix<double, 4, 4> T_rotation_extra = Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
    return T_rotation_extra;
};

Eigen::Matrix<double,1,9> get_error(Eigen::Matrix<double,4,4> moving_to_fixed){
    Eigen::Matrix<double,3,9> ct_points;
    ct_points << 51.0910 ,  45.7005 ,  18.7328  , -26.3736 , -53.1790 , -53.1238 , -38.7812  ,  5.9589 , -26.4164 ,
                80.3660 , 128.6975 , 156.8359 , 158.2443 , 127.2275 ,  87.3330 ,  67.2878 ,  99.4279 , 129.4901 ,
                517.6372 , 516.1141 , 514.3752 , 514.4684 , 515.4078 , 517.3948 , 519.6219 , 505.9474 , 509.5589;


    Eigen::Matrix<double,3,9> world_points ;
    world_points  << -674.874, -714.276, -741.764, -741.121, -716.382, -677.195, -660.926, -683.793, -713.982 ,
                    90.9651,  88.4241,   58.843,   15.514, -11.3519, -9.55646,  3.30793,  50.3275,  16.0374 ,
                    31.3693,   8.2447, -2.56189, -5.17164,  11.5598,    26.49,  36.5624,  10.8589,  2.87831;

    Eigen::Matrix<double,3,9> moved = (moving_to_fixed.block<3,3>(0,0)*ct_points).colwise()+moving_to_fixed.block<3,1>(0,3);

    Eigen::Matrix<double,3,9> error = world_points-moved;
    Eigen::Array<double,1,9> rooted = error.array().square().colwise().sum().sqrt();
    return rooted.matrix();
}

// Lamda to get a rotation matrix from a given angle but with a flipped principal direction
auto transform_flipped_principal_component = [](double alpha)
{
    Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
    poorly_constrained_direction << 1.0, 0.0, 0.0,
        0.0, std::cos(alpha), -std::sin(alpha),
        0.0, std::sin(alpha), std::cos(alpha);

    Eigen::Matrix<double, 3, 3> flipping_direction;
    flipping_direction << -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, 1.0;

    Eigen::Matrix<double, 4, 4> T_rotation_extra = Eigen::Matrix<double, 4, 4>::Identity();
    T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction * flipping_direction;
    return T_rotation_extra;
};

enum ExtractionSurfaceTransformation
{
    ELIMINATE_VERTICIES_WITH_OUTWARD_NORMALS,
    ELIMINATE_VERTICIES_WITH_INNARD_NORMALS,
    SUBSAMPLE_VERTCIES,
    LEAVE_UNCHANGED
};

template <bool is_in_debug>
struct ExtractionSurfaceInfo
{
    size_t connected_components;
    double frequency;
    std::string appendix;
    double reduction_factor;
    ExtractionSurfaceTransformation transform_surface = LEAVE_UNCHANGED;
    int buffer_of_mask;

    ExtractionSurfaceInfo(size_t in_connected_components,
                          double in_frequency,
                          std::string in_appendix,
                          double in_reduction_factor,
                          int in_buffer_of_mask) : connected_components{in_connected_components},
                                                   frequency{in_frequency},
                                                   appendix{in_appendix},
                                                   reduction_factor{in_reduction_factor},
                                                   buffer_of_mask{in_buffer_of_mask} {};
};

template <bool is_in_debug>
std::tuple<Eigen::Matrix<double,3,Eigen::Dynamic>, itk::ImageMaskSpatialObject<3>::ImageType::Pointer> extract_point_cloud(itk::Image<float, 3>::Pointer image, const ExtractionSurfaceInfo<is_in_debug> &info)
{
    auto image_to_fill = itk::ImageMaskSpatialObject<3>::ImageType::New();

    auto bluring = itk::BinomialBlurImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>::New();
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

    auto interpolator = itk::LinearInterpolateImageFunction<itk::Image<float, 3>, double>::New();
    auto transform = itk::AffineTransform<double, 3>::New();
    transform->SetIdentity();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>::New();
    resampleFilter->SetInput(bluring->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetOutputDirection(bluring->GetOutput()->GetDirection());
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);

    if constexpr (is_in_debug)
    {
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
    }
    else
        update_ikt_filter(resampleFilter);

    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<itk::Image<float, 3>>;
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
    for (size_t i = 1; i < histogram->Size(); ++i)
    {
        if (cumulative_frequency >= target_frequency)
        {
            threshold_bin = i;
            break;
        }
        cumulative_frequency += histogram->GetFrequency(i);
    }
    auto minMaxCalculator = itk::MinimumMaximumImageCalculator<itk::Image<float, 3>>::New();
    minMaxCalculator->SetImage(resampleFilter->GetOutput());
    minMaxCalculator->Compute();

    HistogramType::MeasurementType thresholdvalue = histogram->GetBinMin(0, threshold_bin);

    auto binary_threshold = itk::BinaryThresholdImageFilter<itk::Image<float, 3>, itk::Image<unsigned char, 3>>::New();
    binary_threshold->SetInput(resampleFilter->GetOutput());
    binary_threshold->SetOutsideValue(0);
    binary_threshold->SetInsideValue(255);
    binary_threshold->SetLowerThreshold(histogram->GetBinMin(0, threshold_bin));
    binary_threshold->SetUpperThreshold(minMaxCalculator->GetMaximum() + 1.0);

    auto connectedComponentFilter = itk::ConnectedComponentImageFilter<itk::Image<unsigned char, 3>, itk::Image<unsigned char, 3>>::New();
    connectedComponentFilter->SetInput(binary_threshold->GetOutput());

    auto relabelFilter = itk::RelabelComponentImageFilter<itk::Image<unsigned char, 3>, itk::Image<unsigned char, 3>>::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());

    auto filtered_image_with_largest_components = itk::ThresholdImageFilter<itk::Image<unsigned char, 3>>::New();
    filtered_image_with_largest_components->SetInput(relabelFilter->GetOutput());
    filtered_image_with_largest_components->ThresholdOutside(1, info.connected_components);
    filtered_image_with_largest_components->SetOutsideValue(255);

    auto final_binary_threshold = itk::BinaryThresholdImageFilter<itk::Image<unsigned char, 3>, itk::Image<unsigned char, 3>>::New();
    final_binary_threshold->SetInput(filtered_image_with_largest_components->GetOutput());
    final_binary_threshold->SetOutsideValue(0);
    final_binary_threshold->SetInsideValue(255);
    final_binary_threshold->SetLowerThreshold(0);
    final_binary_threshold->SetUpperThreshold(info.connected_components);

    if constexpr (is_in_debug)
    {
        auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
        writer->SetInput(final_binary_threshold->GetOutput());
        writer->SetFileName(info.appendix + "_processed_filtered.mha");
        update_ikt_filter(writer);
    }
    else
        update_ikt_filter(final_binary_threshold);

    itk::ImageRegionIteratorWithIndex<itk::Image<unsigned char, 3>> iterator(final_binary_threshold->GetOutput(), final_binary_threshold->GetOutput()->GetLargestPossibleRegion());
    iterator.GoToBegin();
    std::vector<size_t> number_of_points;
    number_of_points.resize(info.connected_components);
    Eigen::Matrix<double,3,Eigen::Dynamic> centroids = Eigen::Matrix<double,3,Eigen::Dynamic>::Zeros(3,info.connected_components);
    while (!iterator.IsAtEnd()){
        for (unsigned char highlighted_region = 0; highlighted_region < info.connected_components; ++highlighted_region)
            if(iterator.Get()==highlighted_region){
                ++number_of_points[highlighted_region];
                auto itk_point = (1e-3)*final_binary_threshold->GetOutput()->TransformIndexToPhysicalPoint(iterator.GetIndex());
                Eigen::Matrix<double,3,1> point{{itk_point[0],itk_point[1],itk_point[2]}};
                centroids.col(highlighted_region) += point;
            }
        ++iterator;
    }

    for(size_t index = 0; index < centroids.cols(); ++index)
        centroids[index] *= (1.0/number_of_points[index])*(1e3);
    return {centroids, image_to_fill};
}

template <typename pixel_type>
int modify_image_with_transform(Eigen::Matrix<double, 4, 4> transform, typename itk::Image<pixel_type, 3>::Pointer image)
{
    itk::Point<double, 3> origin;
    itk::Matrix<double> direction;
    for (size_t row = 0; row < 3; ++row)
    {
        origin[row] = transform(row, 3);
        for (size_t col = 0; col < 3; ++col)
            direction(row, col) = transform(row, col);
    }
    image->SetOrigin(origin);
    image->SetDirection(direction);
    return 1;
};

template <typename pixel_type>
void print_image_with_transform(typename itk::Image<pixel_type, 3>::Pointer image, const std::string &image_path)
{
    auto writer = itk::ImageFileWriter<typename itk::Image<pixel_type, 3>>::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    update_ikt_filter(writer);
    return;
};

template <typename PixelType>
struct info_solve_registration
{
    using ImageType = itk::Image<PixelType, 3>;
    typename ImageType::ConstPointer fixed_image;
    typename ImageType::ConstPointer moving_image;
    std::optional<itk::ImageMaskSpatialObject<3>::ConstPointer> fixed_image_mask;
    std::optional<itk::ImageMaskSpatialObject<3>::ConstPointer> moving_image_mask;
    const Eigen::Matrix<double, 4, 4> &initial_rotation;
};

constexpr size_t size_info = 3;

struct RegistrationParameters
{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t, size_info> piramid_sizes;
    std::array<double, size_info> bluering_sizes;

    RegistrationParameters(size_t in_bin_numbers,
                           double in_relative_scales,
                           double in_learning_rate,
                           double in_sampling_percentage,
                           double in_relaxation_factor,
                           size_t in_convergence_window_size,
                           size_t in_optimization_iterations,
                           std::array<size_t, size_info> in_piramid_sizes,
                           std::array<double, size_info> in_bluering_sizes) : bin_numbers{in_bin_numbers},
                                                                              relative_scales{in_relative_scales},
                                                                              learning_rate{in_learning_rate},
                                                                              sampling_percentage{in_sampling_percentage},
                                                                              relaxation_factor{in_relaxation_factor},
                                                                              convergence_window_size{in_convergence_window_size},
                                                                              optimization_iterations{in_optimization_iterations},
                                                                              piramid_sizes{in_piramid_sizes},
                                                                              bluering_sizes{in_bluering_sizes}
    {
    }
};

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
    Execute(itk::Object *object, const itk::EventObject &event) override
    {
        if (!(itk::MultiResolutionIterationEvent().CheckEvent(&event)))
        {
            return;
        }
        auto registration = static_cast<RegistrationPointer>(object);
        auto optimizer =
            static_cast<OptimizerPointer>(registration->GetModifiableOptimizer());
        if (registration->GetCurrentLevel() == 0)
        {
            optimizer->SetLearningRate(0.1);
            optimizer->SetMinimumStepLength(0.1);
            optimizer->SetMaximumStepSizeInPhysicalUnits(0.2);
        }
        else
        {
            optimizer->SetLearningRate(optimizer->GetCurrentStepLength());
            optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() *0.2);
            optimizer->SetMaximumStepSizeInPhysicalUnits(optimizer->GetMaximumStepSizeInPhysicalUnits()*0.2);
        }
    }

    void
    Execute(const itk::Object *, const itk::EventObject &) override
    {
        return;
    }
};

template <typename PixelType>
std::tuple<double, Eigen::Matrix<double, 4, 4>, Eigen::Matrix<double, 4, 4>> solve_registration(const info_solve_registration<PixelType> &info_registration, const RegistrationParameters &parameters)
{
    using MaskType = itk::ImageMaskSpatialObject<3>;
    using ImageType = typename info_solve_registration<PixelType>::ImageType;
    using ImageRegistrationType = typename info_solve_registration<PixelType>::ImageType;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using RegistrationType = itk::ImageRegistrationMethodv4<ImageRegistrationType, ImageRegistrationType, TransformType>;

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    typename InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    typename InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

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
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    itk::Vector<double, 3> origin;
    itk::Matrix<double> direction;

    for (size_t row = 0; row < 3; ++row)
    {
        origin[row] = info_registration.initial_rotation(row, 3);
        for (size_t col = 0; col < 3; ++col)
            direction(row, col) = info_registration.initial_rotation(row, col);
    }

    Eigen::Matrix<double, 4, 4> transformation = info_registration.initial_rotation;

    if (!(transformation.block<3, 3>(0, 0).transpose() * transformation.block<3, 3>(0, 0)).isDiagonal())
    {
        std::cout << "failure to initialize rotation matrix...\n";
        std::cout << "values are: \n";
        std::cout << transformation.block<3, 3>(0, 0) << std::endl;
        std::cout << "the multiplication with itself is:";
        std::cout << transformation.block<3, 3>(0, 0).transpose() * transformation.block<3, 3>(0, 0) << std::endl;
        throw std::runtime_error("failure to initialize the rotation matrix");
    }
    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);

    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(initialTransform->GetNumberOfParameters());

    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = 1.0/parameters.relative_scales;
    optimizerScales[4] = 1.0/parameters.relative_scales;
    optimizerScales[5] = 1.0/parameters.relative_scales;

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

    for (size_t i = 0; i < size_info; ++i)
    {
        shrinkFactorsPerLevel[i] = parameters.piramid_sizes[i];
        smoothingSigmasPerLevel[i] = parameters.bluering_sizes[i];
    }

    registration->SetNumberOfLevels(size_info);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    if (parameters.sampling_percentage > 0.99)
    {
        typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::NONE;
        registration->SetMetricSamplingStrategy(samplingStrategy);
    }
    else
    {
        typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::REGULAR;
        registration->MetricSamplingReinitializeSeed(std::rand());
        registration->SetMetricSamplingStrategy(samplingStrategy);
        registration->SetMetricSamplingPercentage(parameters.sampling_percentage);
    }

    registration->SetInPlace(false);

    using CommandType = RegistrationInterfaceCommand<RegistrationType>;
    auto command = CommandType::New();
    registration->AddObserver(itk::MultiResolutionIterationEvent(), command);

    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return {100.0, Eigen::Matrix<double, 4, 4>::Identity(), info_registration.initial_rotation};
    }
    TransformType::Pointer final_registration = registration->GetModifiableTransform();
    Eigen::Matrix<double, 4, 4> final_transformation = Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t row = 0; row < 3; ++row)
    {
        final_transformation(row, 3) = final_registration->GetOffset()[row];
        for (size_t col = 0; col < 3; ++col)
            final_transformation(row, col) = final_registration->GetMatrix()(row, col);
    }
    return {optimizer->GetValue(), final_transformation, info_registration.initial_rotation};
}

void write_point_set(const std::string &filename, itk::PointSet<double, 3>::Pointer pointset)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);
    for (itk::PointSet<double, 3>::PointsContainer::Iterator iterator = pointset->GetPoints()->Begin(); iterator < pointset->GetPoints()->End(); ++iterator)
        file << iterator->Value()[0] << " " << iterator->Value()[1] << " " << iterator->Value()[2] << "\n";
    file.close();
}

template <typename PixelType>
double evaluate_mi_with_both_images(const info_solve_registration<PixelType> &info_registration, size_t bin_numbers)
{
    using RegistrationPixelType = PixelType;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image<PixelType, Dimension>;
    using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
    using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
    using TransformType = itk::VersorRigid3DTransform<double>;

    typename InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    typename InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

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

    TransformType::ParametersType displacement(initialTransform->GetNumberOfParameters());
    displacement.Fill(0);
    try
    {
        metric->Initialize();
        metric->SetParameters(displacement);

        return metric->GetValue();
    }
    catch (const itk::ExceptionObject &err)
    {
        return 100.0;
    }
}

int main(int argc, char * argv[])
{
    if(argc!=3){
        std::cout << "need to call function with 3 params\n";
        return 1;
    }

    auto image_reader_fixed = itk::ImageFileReader<itk::Image<float, 3>>::New();
    image_reader_fixed->SetFileName(argv[1]);
    update_ikt_filter(image_reader_fixed);
    auto [centroids_fixed, mask_fixed_image] = extract_point_cloud(image_reader_fixed->GetOutput(), ExtractionSurfaceInfo<true>{3, 0.95, "fixed", 5, 5});

    auto image_reader_moving = itk::ImageFileReader<itk::Image<float, 3>>::New();
    image_reader_moving->SetFileName(argv[2]);
    update_ikt_filter(image_reader_moving);
    auto [centroids_moving, mask_moving_image] = extract_point_cloud(image_reader_moving->GetOutput(), ExtractionSurfaceInfo<true>{3, 0.8, "moving", 5, 5});

    Eigen::Matrix<double, 4, 4> best_transformation_icp;

    Eigen::Matrix<double, 4, 4> Timage_origin_fixed = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> Timage_origin_moving = Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t row = 0; row < 3; ++row)
    {
        Timage_origin_fixed(row, 3) = image_reader_fixed->GetOutput()->GetOrigin()[row];
        Timage_origin_moving(row, 3) = image_reader_moving->GetOutput()->GetOrigin()[row];
        for (size_t col = 0; col < 3; ++col)
        {
            Timage_origin_fixed(row, col) = image_reader_fixed->GetOutput()->GetDirection()(row, col);
            Timage_origin_moving(row, col) = image_reader_moving->GetOutput()->GetDirection()(row, col);
        }
    }

    auto converter = [](itk::Image<float, 3>::ConstPointer image)
    {
        auto rescaler = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>::New();
        rescaler->SetInput(image);
        rescaler->SetOutputMinimum(0);
        rescaler->SetOutputMaximum(itk::NumericTraits<unsigned char>::max());
        update_ikt_filter(rescaler);
        itk::Image<float, 3>::Pointer output = rescaler->GetOutput();
        return output;
    };

    auto fixed = converter(image_reader_fixed->GetOutput());
    auto moving = converter(image_reader_moving->GetOutput());

    modify_image_with_transform<float>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, fixed);
    modify_image_with_transform<float>(best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, mask_fixed_image);
    modify_image_with_transform<unsigned char>(best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, mask_moving_image);

    std::ofstream myfile{"results_of_fullscale_optimization.csv"};
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    // Optimizer parameters
    constexpr size_t local_permut = 1;

    std::array<size_t, local_permut> bin_numbers{100};
    std::array<double, local_permut> percentage_numbers{1.0};
    std::array<double, local_permut> relative_scales{1000.0};
    std::array<double, local_permut> learning_rate{.1};
    std::array<double, local_permut> relaxation_factor{0.7};
    std::array<size_t, local_permut> optimization_iterations{1000};
    std::array<size_t, local_permut> convergence_window_size{40};

    std::array<std::array<size_t, size_info>, local_permut> piramid_sizes{{{3, 2, 1}}};
    std::array<std::array<double, size_info>, local_permut> bluering_sizes{{{2, 0, 0}}};

    constexpr size_t total_permutations = bin_numbers.size() * percentage_numbers.size() * relative_scales.size() * learning_rate.size() * relaxation_factor.size() * convergence_window_size.size() * piramid_sizes.size() * bluering_sizes.size();
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
                                        for (const auto &blur_size : bluering_sizes)
                                        {
                                            curan::utilities::Job job{"solving registration", [&]()
                                                                      {
                                                                          std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                                                          auto [cost, transformation, initial_transform] = solve_registration(info_solve_registration<float>{fixed,
                                                                                                                                                                                     moving,
                                                                                                                                                                                     transformed_mask_fixed_image,
                                                                                                                                                                                     transformed_mask_moving_image,
                                                                                                                                                                                     Eigen::Matrix<double, 4, 4>::Identity()},
                                                                                                                                              RegistrationParameters{bin_n,
                                                                                                                                                                     rel_scale,
                                                                                                                                                                     learn_rate,
                                                                                                                                                                     percent_n,
                                                                                                                                                                     relax_factor,
                                                                                                                                                                     wind_size,
                                                                                                                                                                     iters,
                                                                                                                                                                     pira_size,
                                                                                                                                                                     blur_size});
                                                                          std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                                                          {
                                                                              std::lock_guard<std::mutex> g{mut};
                                                                              myfile << total_runs << "," << bin_n << "," << percent_n << "," << rel_scale << "," << learn_rate << "," << relax_factor << "," << wind_size << ", {";
                                                                              for (const auto &val : pira_size)
                                                                                  myfile << val << " ";
                                                                              myfile << "}, {";
                                                                              for (const auto &val : blur_size)
                                                                                  myfile << val << " ";
                                                                              myfile << "}," << cost << "," << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;

                                                                              ++total_runs;
                                                                              full_runs.emplace_back(cost, transformation);
                                                                              std::printf("mi (%.2f %%)\n", (total_runs / (double)total_permutations) * 100);
                                                                          }
                                                                      }};
                                            pool->submit(job);
                                        }
    }

    std::sort(full_runs.begin(), full_runs.end(), [](const std::tuple<double, Eigen::Matrix4d> &a, const std::tuple<double, Eigen::Matrix4d> &b)
              { return std::get<0>(a) < std::get<0>(b); });
    const double pi = std::atan(1) * 4;
    auto [cost, best_transformation_mi] = full_runs[0];

    std::cout << "\n error with MI:" <<  get_error(transformation_acording_to_pca_fixed * best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse());
    std::cout << "\n error with ICP:" << get_error(transformation_acording_to_pca_fixed * best_transformation_icp * transformation_acording_to_pca_moving.inverse()) << "\n\n\n";


    auto return_current_time_and_date = [](){
	    auto now = std::chrono::system_clock::now();
    	auto in_time_t = std::chrono::system_clock::to_time_t(now);

    	std::stringstream ss;
    	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
   		return ss.str();
	};

	//Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");
	//std::stringstream optimized_values;
	//optimized_values << (transformation_acording_to_pca_fixed * best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse()).format(CleanFmt) << std::endl;
	
	// Once the optimization is finished we need to print a json file with the correct configuration of the image transformation to the 
	// tracker transformation ()
	//std::printf("\nRememeber that you always need to\nperform the temporal calibration before attempting the\nspacial calibration! Produced JSON file:\n");

	//nlohmann::json registration_data;
	//registration_data["timestamp"] = return_current_time_and_date();
	//registration_data["moving_to_fixed_transform"] = optimized_values.str();
	//registration_data["registration_error"] = evaluate_mi_with_both_images(info_solve_registration<float>{fixed, moving, transformed_mask_fixed_image, transformed_mask_moving_image, Eigen::Matrix<double, 4, 4>::Identity()}, bin_numbers[0]);
    //registration_data["type"] = "MI";

	// write prettified JSON to another file
	//std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/registration.json");
	//o << registration_data;
	//std::cout << registration_data << std::endl;

    return 0;
}