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

constexpr size_t number_of_connected_components = 6;
constexpr double frequency_of_bone_occurence = 0.85;

struct ExtractionSurfaceInfo{
    size_t connected_components;
    double frequency;
    std::string appendix;
    ExtractionSurfaceInfo(size_t in_connected_components,
                      double in_frequency,
                      std::string in_appendix) : connected_components{in_connected_components} ,
                                                 frequency{in_frequency},
                                                 appendix{in_appendix}
    {};
};

itk::Mesh<double>::Pointer extract_point_cloud(itk::Image<double, 3>::Pointer image,const ExtractionSurfaceInfo& info)
{
    auto bluring = itk::BinomialBlurImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
    bluring->SetInput(image);
    bluring->SetRepetitions(10);

    update_ikt_filter(bluring);

    constexpr double reduction_factor = 5.0;
    auto input_size = image->GetLargestPossibleRegion().GetSize();
    auto input_spacing = image->GetSpacing();
    auto input_origin = image->GetOrigin();

    double physicalspace[3];
    physicalspace[0] = input_size[0] * input_spacing[0];
    physicalspace[1] = input_size[1] * input_spacing[1];
    physicalspace[2] = input_size[2] * input_spacing[2];

    auto output_size = input_size;
    output_size[0] = (size_t)std::floor((1.0 / reduction_factor) * output_size[0]);
    output_size[1] = (size_t)std::floor((1.0 / reduction_factor) * output_size[1]);
    output_size[2] = (size_t)std::floor((1.0 / reduction_factor) * output_size[2]);

    auto output_spacing = input_spacing;
    output_spacing[0] = physicalspace[0] / output_size[0];
    output_spacing[1] = physicalspace[1] / output_size[1];
    output_spacing[2] = physicalspace[2] / output_size[2];

    auto interpolator = itk::LinearInterpolateImageFunction<itk::Image<double, 3>, double>::New();
    auto transform = itk::AffineTransform<double, 3>::New();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
    resampleFilter->SetInput(bluring->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);

    {
        auto writer = itk::ImageFileWriter<itk::Image<double, 3>>::New();
        writer->SetInput(resampleFilter->GetOutput());
        writer->SetFileName(info.appendix+"_processed.mha");
        update_ikt_filter(writer);
    }

    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<itk::Image<double, 3>>;
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
    auto minMaxCalculator = itk::MinimumMaximumImageCalculator<itk::Image<double, 3>>::New();
    minMaxCalculator->SetImage(resampleFilter->GetOutput());
    minMaxCalculator->Compute();

    HistogramType::MeasurementType thresholdvalue = histogram->GetBinMin(0, threshold_bin);

    auto binary_threshold = itk::BinaryThresholdImageFilter<itk::Image<double, 3>, itk::Image<unsigned char, 3>>::New();
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

    {
        auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
        writer->SetInput(final_binary_threshold->GetOutput());
        writer->SetFileName(info.appendix+"_processed_filtered.mha");
        update_ikt_filter(writer);
    }

    auto meshSource = itk::BinaryMask3DMeshSource<itk::Image<unsigned char, 3>, itk::Mesh<double>>::New();
    meshSource->SetObjectValue(255);
    meshSource->SetInput(final_binary_threshold->GetOutput());
    update_ikt_filter(meshSource);

    {
        using WriterType = itk::MeshFileWriter<itk::Mesh<double>>;
        auto writer = WriterType::New();
        writer->SetFileName(info.appendix+"_point_cloud.obj");
        writer->SetInput(meshSource->GetOutput());
        update_ikt_filter(writer);
    }

    return meshSource->GetOutput();
}

std::tuple<Eigen::Matrix<double,4,4>,itk::PointSet<double, 3>::Pointer> recentered_data(itk::Mesh<double>::Pointer mesh){
    itk::PointSet<double, 3>::Pointer point_set = itk::PointSet<double, 3>::New();
    auto point_container = itk::PointSet<double, 3>::PointsContainer::New();

    Eigen::Matrix<double,4,4> pca_alignement = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, 3> points_in_matrix_form = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(mesh->GetNumberOfPoints(), 4);
    using PointsIterator = itk::Mesh<double>::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    PointsIterator end = mesh->GetPoints()->End();
    size_t index = 0;
    while (pointIterator != end)
    {
        auto p = pointIterator->Value();
        points_in_matrix_form(index, 0) = p[0];
        points_in_matrix_form(index, 1) = p[1];
        points_in_matrix_form(index, 2) = p[2];
        ++pointIterator;
        ++index;
    }

    Eigen::Matrix<double, 3, 1> center_of_moving_image = points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double, 1, 3> to_subtract = center_of_moving_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;
    Eigen::Matrix<double, 3, 3> covariance_of_moving_surface = points_in_matrix_form.transpose() * points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(covariance_of_moving_surface, Eigen::ComputeFullV | Eigen::ComputeFullU);
    assert(svd.computeU());
    auto rotation_of_moving_image = svd.matrixU();
    rotation_of_moving_image.col(2) = rotation_of_moving_image.col(0).cross(rotation_of_moving_image.col(1));
    pca_alignement.block<3, 3>(0, 0) = rotation_of_moving_image;
    pca_alignement.block<3,1>(0, 3) = center_of_moving_image;

    Eigen::Matrix<double,4,4> inv_pca_alignement = pca_alignement;
    inv_pca_alignement.block<3, 3>(0, 0) = rotation_of_moving_image.transpose();
    inv_pca_alignement.block<3,1>(0, 3) = -rotation_of_moving_image.transpose()*center_of_moving_image;

    pointIterator = mesh->GetPoints()->Begin();
    end = mesh->GetPoints()->End();
    index = 0;

    while (pointIterator != end)
    {
        const auto& p = pointIterator->Value();
        Eigen::Matrix<double,4,1> original_point = Eigen::Matrix<double,4,1>::Ones();

        original_point[0] = p[0];
        original_point[1] = p[1];
        original_point[2] = p[2];

        Eigen::Matrix<double,4,1> transformed_point = inv_pca_alignement*original_point;

        itk::PointSet<double, 3>::PointType p_c;
        p_c[0] = transformed_point[0];
        p_c[1] = transformed_point[1];
        p_c[2] = transformed_point[2];
        point_container->InsertElement(index,p_c);
        ++pointIterator;
        ++index;
    } 
    point_set->SetPoints(point_container);
    return {pca_alignement,point_set};
}

void extract_surface(itk::Image<double, 3>::Pointer image)
{
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

std::tuple<double, Eigen::Matrix<double, 4, 4>> icp_registration(Eigen::Matrix4d initial_config, itk::PointSet<double, 3>::Pointer fixed_point_cloud, itk::PointSet<double, 3>::Pointer moving_point_cloud )
{
    auto metric = itk::EuclideanDistancePointMetric<itk::PointSet<double, 3>, itk::PointSet<double, 3>>::New();
    auto transform = itk::Euler3DTransform<double>::New();
    auto optimizer = itk::LevenbergMarquardtOptimizer::New();
    optimizer->SetUseCostFunctionGradient(false);
    
    auto registration = itk::PointSetToPointSetRegistrationMethod<itk::PointSet<double, 3>, itk::PointSet<double, 3>>::New();
    itk::LevenbergMarquardtOptimizer::ScalesType scales(transform->GetNumberOfParameters());
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
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    itk::Vector<double, 3> origin;
    itk::Matrix<double> direction;

    for (size_t row = 0; row < 3; ++row){
        origin[row] = initial_config(row, 3);
        for (size_t col = 0; col < 3; ++col)
            direction(row, col) = initial_config(row, col);
    }

    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);

    registration->SetInitialTransformParameters(initialTransform->GetParameters());
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
    registration->SetTransform(transform);
    registration->SetFixedPointSet(fixed_point_cloud);
    registration->SetMovingPointSet(moving_point_cloud);

    update_ikt_filter(registration);

    Eigen::Matrix<double, 4, 4> final_transformation = Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t row = 0; row < 3; ++row){
        final_transformation(row, 3) = transform->GetOffset()[row];
        for (size_t col = 0; col < 3; ++col)
            final_transformation(row, col) = transform->GetMatrix()(row, col);
    }

    return {optimizer->GetValue().two_norm(), final_transformation};
}

    //"C:/Dev/Curan/build/bin/resources/us_image1_cropepd_volume.mha"
    // "C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha"

    //  "C:/Dev/Curan/build/bin/resources/ct_image1_full_volume.mha"
    // "C:/Dev/Curan/build/bin/resources/ct_image1_cropepd_volume.mha"


int main()
{
    auto image_reader_fixed = itk::ImageFileReader<itk::Image<double, 3>>::New();
    image_reader_fixed->SetFileName("C:/Dev/Curan/build/bin/resources/us_image1_cropepd_volume.mha");
    update_ikt_filter(image_reader_fixed);
    auto point_cloud_fixed = extract_point_cloud(image_reader_fixed->GetOutput(),ExtractionSurfaceInfo{6,0.85,"fixed"});
    auto [transformation_acording_to_pca_fixed,fixed_point_set] = recentered_data(point_cloud_fixed);

    auto image_reader_moving = itk::ImageFileReader<itk::Image<double, 3>>::New();
    image_reader_moving->SetFileName("C:/Dev/Curan/build/bin/resources/ct_image1_cropepd_volume.mha");
    update_ikt_filter(image_reader_moving);
    auto point_cloud_moving = extract_point_cloud(image_reader_moving->GetOutput(),ExtractionSurfaceInfo{6,0.85,"moving"});
    auto [transformation_acording_to_pca_moving,moving_point_set] = recentered_data(point_cloud_moving);

    std::vector<Eigen::Matrix<double, 4, 4>> initial_guesses_icp;
    for (double angle = 0; angle < 360.0; angle += 10.0)
    {
        initial_guesses_icp.push_back(transform_x(rad2deg(angle)));
        initial_guesses_icp.push_back(transform_flipped_principal_component(rad2deg(angle)));
    }

    auto run_parameterized_icp_optimization = [&]()
    {
        std::vector<std::tuple<double, Eigen::Matrix<double, 4, 4>>> full_runs_inner;
        {
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6, curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            size_t counter = 0;
            for (const auto &initial_config : initial_guesses_icp)
            {
                curan::utilities::Job job{"solving icp", [&](){
                    auto solution = icp_registration(initial_config,fixed_point_set,moving_point_set);
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs_inner.emplace_back(solution);
                        ++counter;
                        std::printf("%.0f %% %.3f\n", (counter / (double)initial_guesses_icp.size()) * 100, std::get<0>(solution));
                    }
                }};
                pool->submit(job);
            }
        }
        return full_runs_inner;
    };
    
    return 0;
}