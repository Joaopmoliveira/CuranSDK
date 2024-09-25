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



std::tuple<Eigen::Matrix<double,4,9>,Eigen::Matrix<double,4,9>> get_points_to_minimize(){
    Eigen::Matrix<double,4,9> points_in_us_coordinates;
    points_in_us_coordinates << 1;

    Eigen::Matrix<double,4,9> points_in_ct_coordinates ;
    points_in_ct_coordinates << 1;

    return {points_in_us_coordinates,points_in_ct_coordinates};
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
std::tuple<itk::Mesh<double>::Pointer, itk::ImageMaskSpatialObject<3>::ImageType::Pointer> extract_point_cloud(itk::Image<double, 3>::Pointer image, const ExtractionSurfaceInfo<is_in_debug> &info)
{
    auto image_to_fill = itk::ImageMaskSpatialObject<3>::ImageType::New();

    auto bluring = itk::BinomialBlurImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
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

    auto interpolator = itk::LinearInterpolateImageFunction<itk::Image<double, 3>, double>::New();
    auto transform = itk::AffineTransform<double, 3>::New();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
    resampleFilter->SetInput(bluring->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);

    if constexpr (is_in_debug)
    {
        auto writer = itk::ImageFileWriter<itk::Image<double, 3>>::New();
        writer->SetInput(resampleFilter->GetOutput());
        writer->SetFileName(info.appendix + "_processed.mha");
        update_ikt_filter(writer);
    }
    else
        update_ikt_filter(resampleFilter);

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

    if constexpr (is_in_debug)
    {
        auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
        writer->SetInput(final_binary_threshold->GetOutput());
        writer->SetFileName(info.appendix + "_processed_filtered.mha");
        update_ikt_filter(writer);
    }
    else
        update_ikt_filter(final_binary_threshold);

    // we extract the mask as a function of the number of connected components we wish to extract
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
        for (unsigned char highlighted_region = 0; highlighted_region < info.connected_components; ++highlighted_region)
        {
            minimum_x_indicies_of_regions[highlighted_region] = 10000;
            maximum_x_indicies_of_regions[highlighted_region] = 0;
            minimum_y_indicies_of_regions[highlighted_region] = 10000;
            maximum_y_indicies_of_regions[highlighted_region] = 0;
            minimum_z_indicies_of_regions[highlighted_region] = 10000;
            maximum_z_indicies_of_regions[highlighted_region] = 0;
        }

        itk::ImageRegionIteratorWithIndex<itk::Image<unsigned char, 3>> iterator(relabelFilter->GetOutput(), relabelFilter->GetOutput()->GetLargestPossibleRegion());
        iterator.GoToBegin();

        while (!iterator.IsAtEnd())
        {
            for (int highlighted_region = 0; highlighted_region < info.connected_components; ++highlighted_region)
            {
                if ((int)iterator.Get() == (highlighted_region + 1))
                {
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

        itk::ImageMaskSpatialObject<3>::ImageType::SizeType size = relabelFilter->GetOutput()->GetLargestPossibleRegion().GetSize();
        itk::ImageMaskSpatialObject<3>::ImageType::IndexType index = {{0, 0, 0}};
        itk::ImageMaskSpatialObject<3>::ImageType::RegionType region;
        region.SetSize(size);
        region.SetIndex(index);
        image_to_fill->SetRegions(region);
        image_to_fill->Allocate(true);
        image_to_fill->SetSpacing(relabelFilter->GetOutput()->GetSpacing());
        image_to_fill->SetOrigin(relabelFilter->GetOutput()->GetOrigin());
        image_to_fill->SetDirection(relabelFilter->GetOutput()->GetDirection());
        itk::ImageRegionIteratorWithIndex<itk::Image<unsigned char, 3>> iterator_of_image_to_fill(image_to_fill, image_to_fill->GetLargestPossibleRegion());
        while (!iterator_of_image_to_fill.IsAtEnd())
        {
            auto index = iterator_of_image_to_fill.GetIndex();
            for (unsigned char highlighted_region = 0; highlighted_region < info.connected_components; ++highlighted_region)
            {
                bool is_inside_highlighted = (index[0] > minimum_x_indicies_of_regions[highlighted_region] - info.buffer_of_mask && index[0] < info.buffer_of_mask + maximum_x_indicies_of_regions[highlighted_region]) &&
                                             (index[1] > minimum_y_indicies_of_regions[highlighted_region] - info.buffer_of_mask && index[1] < info.buffer_of_mask + maximum_y_indicies_of_regions[highlighted_region]) &&
                                             (index[2] > minimum_z_indicies_of_regions[highlighted_region] - info.buffer_of_mask && index[2] < info.buffer_of_mask + maximum_z_indicies_of_regions[highlighted_region]);
                if (is_inside_highlighted)
                {
                    iterator_of_image_to_fill.Set(255);
                    break;
                }
            }
            ++iterator_of_image_to_fill;
        }

        if constexpr (is_in_debug)
        {
            auto writer = itk::ImageFileWriter<itk::Image<unsigned char, 3>>::New();
            writer->SetInput(image_to_fill);
            writer->SetFileName(info.appendix + "_filled_mask.mha");
            update_ikt_filter(writer);
        }
    }

    auto meshSource = itk::BinaryMask3DMeshSource<itk::Image<unsigned char, 3>, itk::Mesh<double>>::New();
    meshSource->SetObjectValue(255);
    meshSource->SetInput(final_binary_threshold->GetOutput());
    update_ikt_filter(meshSource);

    if constexpr (is_in_debug)
    {
        using WriterType = itk::MeshFileWriter<itk::Mesh<double>>;
        auto writer = WriterType::New();
        writer->SetFileName(info.appendix + "_point_cloud.obj");
        writer->SetInput(meshSource->GetOutput());
        update_ikt_filter(writer);
    }

    return {meshSource->GetOutput(), image_to_fill};
}

itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::Pointer convert_mesh(const itk::Mesh<double>::Pointer &mesh)
{
    class MeshTriangleVisitor
    {
    public:
        itk::Mesh<double>::Pointer inmesh;
        using MeshSourceType = itk::AutomaticTopologyMeshSource<itk::Mesh<double>>;
        itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer container;

        void set_required_data(itk::Mesh<double>::Pointer ref_inmesh)
        {
            inmesh = ref_inmesh;
            container = itk::QuadEdgeMesh<double, 3>::PointsContainer::New();
            container->Reserve(inmesh->GetNumberOfPoints());
        }

        using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
        void
        Visit(unsigned long cellId, TriangleType *t)
        {
            TriangleType::PointIdIterator pit = t->PointIdsBegin();
            for (size_t index = 0; pit != t->PointIdsEnd(); ++pit, ++index)
            {
                container->SetElement(*pit, inmesh->GetPoint(*pit));
            }
        }

        MeshTriangleVisitor() = default;
        virtual ~MeshTriangleVisitor() = default;
    };

    class MeshTriangleConstructorVisitor
    {
    public:
        itk::Mesh<double>::Pointer inmesh;
        using MeshSourceType = itk::AutomaticTopologyMeshSource<itk::Mesh<double>>;
        itk::QuadEdgeMesh<double, 3>::Pointer outmesh;

        void set_required_data(itk::Mesh<double>::Pointer ref_inmesh, itk::QuadEdgeMesh<double, 3>::Pointer ref_outmesh, itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer in_container)
        {
            inmesh = ref_inmesh;
            outmesh = ref_outmesh;
            outmesh->SetPoints(in_container);
        }

        using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
        void
        Visit(unsigned long cellId, TriangleType *t)
        {
            TriangleType::PointIdIterator pit = t->PointIdsBegin();
            std::array<TriangleType::PointIdentifier, 3> ident;
            for (size_t index = 0; pit != t->PointIdsEnd(); ++pit, ++index)
                ident[index] = *pit;
            outmesh->AddFaceTriangle(ident[0], ident[1], ident[2]);
        }

        MeshTriangleConstructorVisitor() = default;
        virtual ~MeshTriangleConstructorVisitor() = default;
    };

    itk::QuadEdgeMesh<double, 3>::Pointer out_mesh = itk::QuadEdgeMesh<double, 3>::New();
    itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer container;
    {
        using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
        using TriangleVisitorInterfaceType = itk::CellInterfaceVisitorImplementation<itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits, TriangleType, MeshTriangleVisitor>;
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
        using TriangleVisitorInterfaceType = itk::CellInterfaceVisitorImplementation<itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits, TriangleType, MeshTriangleConstructorVisitor>;
        auto triangleVisitor = TriangleVisitorInterfaceType::New();
        triangleVisitor->set_required_data(mesh, out_mesh, container);
        using CellMultiVisitorType = itk::Mesh<double>::CellType::MultiVisitor;
        auto multiVisitor = CellMultiVisitorType::New();
        multiVisitor->AddVisitor(triangleVisitor);
        mesh->Accept(multiVisitor);
    }
    auto computation_of_normals = itk::NormalQuadEdgeMeshFilter<itk::QuadEdgeMesh<double, 3>, itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::New();
    itk::NormalQuadEdgeMeshFilter<itk::QuadEdgeMesh<double, 3>, itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::WeightEnum weight_type = itk::NormalQuadEdgeMeshFilterEnums::Weight::THURMER;
    computation_of_normals->SetInput(out_mesh);
    computation_of_normals->SetWeight(weight_type);
    update_ikt_filter(computation_of_normals);
    return computation_of_normals->GetOutput();
}

itk::PointSet<double, 3>::Pointer prune_surface(itk::Mesh<double>::Pointer mesh)
{
    auto converted_mesh = convert_mesh(mesh);
    auto point_container = itk::PointSet<double, 3>::PointsContainer::New();
    itk::PointSet<double, 3>::Pointer point_set = itk::PointSet<double, 3>::New();
    itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::PointsContainerIterator p_it = converted_mesh->GetPoints()->Begin();
    itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::PointDataContainerIterator d_it = converted_mesh->GetPointData()->Begin();
    size_t index = 0;
    while (p_it != converted_mesh->GetPoints()->End())
    {
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
        if (normal.transpose() * point > 0.23)
        {
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

template <bool is_in_debug>
itk::Mesh<double>::Pointer extract_surface(itk::Image<double, 3>::Pointer image, const ExtractionSurfaceInfo<is_in_debug> &info)
{
}

std::tuple<Eigen::Matrix<double, 4, 4>, itk::PointSet<double, 3>::Pointer> recentered_data(const itk::Mesh<double>::Pointer &mesh)
{
    std::cout << "Number of points :" << mesh->GetNumberOfPoints() << std::endl;
    Eigen::Matrix<double, 4, 4> pca_alignement = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, 3> points_in_matrix_form = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(mesh->GetNumberOfPoints(), 3);
    using PointsIterator = itk::Mesh<double>::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    size_t index = 0;
    while (pointIterator != mesh->GetPoints()->End())
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
    pca_alignement.block<3, 1>(0, 3) = center_of_moving_image;

    Eigen::Matrix<double, 4, 4> inv_pca_alignement = pca_alignement;
    inv_pca_alignement.block<3, 3>(0, 0) = rotation_of_moving_image.transpose();
    inv_pca_alignement.block<3, 1>(0, 3) = -rotation_of_moving_image.transpose() * center_of_moving_image;

    pointIterator = mesh->GetPoints()->Begin();

    while (pointIterator != mesh->GetPoints()->End())
    {
        auto &p = pointIterator->Value();
        Eigen::Matrix<double, 4, 1> original_point = Eigen::Matrix<double, 4, 1>::Ones();
        original_point[0] = p[0];
        original_point[1] = p[1];
        original_point[2] = p[2];
        Eigen::Matrix<double, 4, 1> transformed_point = inv_pca_alignement * original_point;
        p[0] = transformed_point[0];
        p[1] = transformed_point[1];
        p[2] = transformed_point[2];
        ++pointIterator;
    }
    return {pca_alignement, prune_surface(mesh)};
}

std::tuple<double, Eigen::Matrix<double, 4, 4>> icp_registration(Eigen::Matrix4d initial_config, itk::PointSet<double, 3>::Pointer &fixed_point_cloud, itk::PointSet<double, 3>::Pointer &moving_point_cloud)
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

    for (size_t row = 0; row < 3; ++row)
    {
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
    for (size_t row = 0; row < 3; ++row)
    {
        final_transformation(row, 3) = transform->GetOffset()[row];
        for (size_t col = 0; col < 3; ++col)
            final_transformation(row, col) = transform->GetMatrix()(row, col);
    }

    return {optimizer->GetValue().two_norm(), final_transformation};
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
    optimizerScales[3] = parameters.relative_scales;
    optimizerScales[4] = parameters.relative_scales;
    optimizerScales[5] = parameters.relative_scales;

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

int main()
{
    std::cout << "extracting surface from fixed\n";
    auto image_reader_fixed = itk::ImageFileReader<itk::Image<double, 3>>::New();
    image_reader_fixed->SetFileName("C:/Dev/Curan/build/bin/resources/us_image1_cropepd_volume.mha");
    update_ikt_filter(image_reader_fixed);
    auto [point_cloud_fixed, mask_fixed_image] = extract_point_cloud(image_reader_fixed->GetOutput(), ExtractionSurfaceInfo<false>{3, 0.9, "fixed", 5, 5});
    auto [transformation_acording_to_pca_fixed, tmp_fixed_point_set] = recentered_data(point_cloud_fixed);

    auto fixed_point_set = tmp_fixed_point_set;
    std::cout << "extracting surface from moving\n";
    auto image_reader_moving = itk::ImageFileReader<itk::Image<double, 3>>::New();
    image_reader_moving->SetFileName("C:/Dev/Curan/build/bin/resources/ct_image1_cropepd_volume.mha");
    update_ikt_filter(image_reader_moving);
    auto [point_cloud_moving, mask_moving_image] = extract_point_cloud(image_reader_moving->GetOutput(), ExtractionSurfaceInfo<false>{3, 0.8, "moving", 5, 5});
    auto [transformation_acording_to_pca_moving, tmp_moving_point_set] = recentered_data(point_cloud_moving);
    auto moving_point_set = tmp_moving_point_set;

    std::cout << "setting icp data\n";
    std::vector<Eigen::Matrix<double, 4, 4>> initial_guesses_icp;
    for (double angle = 0; angle < 360.0; angle += 45.0)
    {
        initial_guesses_icp.push_back(transform_x(rad2deg(angle)));
        initial_guesses_icp.push_back(transform_flipped_principal_component(rad2deg(angle)));
    }

    std::cout << "running parallel solving of icp\n";
    std::vector<std::tuple<double, Eigen::Matrix<double, 4, 4>>> full_runs_inner;
    {
        std::mutex mut;
        auto pool = curan::utilities::ThreadPool::create(6, curan::utilities::TERMINATE_ALL_PENDING_TASKS);
        size_t counter = 0;
        for (const auto &initial_config : initial_guesses_icp)
        {
            curan::utilities::Job job{"solving icp", [&]()
                                      {
                                          auto solution = icp_registration(initial_config, fixed_point_set, moving_point_set);
                                          {
                                              std::lock_guard<std::mutex> g{mut};
                                              full_runs_inner.emplace_back(solution);
                                              ++counter;
                                              std::printf("%.2f %% %.3f\n", (counter / (double)initial_guesses_icp.size()) * 100, std::get<0>(solution));
                                          }
                                      }};
            pool->submit(job);
        }
    }

    Eigen::Matrix<double, 4, 4> best_transformation_icp;
    auto min_element_iter = std::min_element(full_runs_inner.begin(), full_runs_inner.end(), [](const auto &a, const auto &b)
                                             { return std::get<0>(a) < std::get<0>(b); });

    if (min_element_iter != full_runs_inner.end())
    {
        auto [minimum, best_icp_transform] = *min_element_iter;
        best_transformation_icp = best_icp_transform;
        std::cout << "Minimum Value: " << minimum << std::endl;
        std::cout << "Corresponding Transformation Matrix (ICP): \n"
                  << best_icp_transform << std::endl;
    }

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

    auto converter = [](itk::Image<double, 3>::ConstPointer image)
    {
        auto rescaler = itk::RescaleIntensityImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
        auto transformer = itk::CastImageFilter<itk::Image<double, 3>, itk::Image<unsigned char, 3>>::New();
        rescaler->SetInput(image);
        rescaler->SetOutputMinimum(0);
        rescaler->SetOutputMaximum(itk::NumericTraits<unsigned char>::max());
        transformer->SetInput(rescaler->GetOutput());
        update_ikt_filter(transformer);
        itk::Image<unsigned char, 3>::Pointer output = transformer->GetOutput();
        return output;
    };

    auto fixed = converter(image_reader_fixed->GetOutput());
    auto moving = converter(image_reader_moving->GetOutput());

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, fixed);
    modify_image_with_transform<unsigned char>((best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);

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
    std::array<std::array<double, size_info>, local_permut> bluering_sizes{{{2, 1, 0}}};

    constexpr size_t total_permutations = bin_numbers.size() * percentage_numbers.size() * relative_scales.size() * learning_rate.size() * relaxation_factor.size() * convergence_window_size.size() * piramid_sizes.size() * bluering_sizes.size();
    std::vector<std::tuple<double, Eigen::Matrix<double, 4, 4>>> full_runs;

    auto transformed_mask_fixed_image = itk::ImageMaskSpatialObject<3>::New();
    transformed_mask_fixed_image->SetImage(mask_fixed_image);
    update_ikt_filter(transformed_mask_fixed_image);
    auto transformed_mask_moving_image = itk::ImageMaskSpatialObject<3>::New();
    transformed_mask_moving_image->SetImage(mask_moving_image);
    update_ikt_filter(transformed_mask_moving_image);

    std::cout << "mi_before_optimization: " << evaluate_mi_with_both_images(info_solve_registration<unsigned char>{fixed, moving, transformed_mask_fixed_image, transformed_mask_moving_image, Eigen::Matrix<double, 4, 4>::Identity()}, bin_numbers[0]) << std::endl;

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
                                                                          auto [cost, transformation, initial_transform] = solve_registration(info_solve_registration<unsigned char>{fixed,
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

    std::cout << "cost of best: " << cost << "\nbest transform according to MI:\n"
              << best_transformation_mi.inverse() << std::endl;

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, fixed);
    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed.inverse() * Timage_origin_fixed, mask_fixed_image);

    modify_image_with_transform<unsigned char>(best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);
    modify_image_with_transform<unsigned char>(best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, mask_moving_image);

    transformed_mask_moving_image->SetImage(mask_moving_image);
    update_ikt_filter(transformed_mask_moving_image);
    std::cout << "mi after optimization: " << evaluate_mi_with_both_images(info_solve_registration<unsigned char>{fixed, moving, transformed_mask_fixed_image, transformed_mask_moving_image, Eigen::Matrix<double, 4, 4>::Identity()}, bin_numbers[0]) << std::endl;

    modify_image_with_transform<unsigned char>(Timage_origin_fixed, fixed);
    print_image_with_transform<unsigned char>(fixed, "fixed_image.mha");

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);
    print_image_with_transform<unsigned char>(moving, "moving_image_after_pca.mha");

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed * best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);
    print_image_with_transform<unsigned char>(moving, "moving_image_after_icp.mha");

    modify_image_with_transform<unsigned char>(transformation_acording_to_pca_fixed * best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse() * Timage_origin_moving, moving);
    print_image_with_transform<unsigned char>(moving, "moving_image_after_icp.mha");

    std::cout << "final transformation:\n"
              << transformation_acording_to_pca_fixed * best_transformation_mi.inverse() * best_transformation_icp * transformation_acording_to_pca_moving.inverse() << std::endl;
    std::cout << "final transformation with icp only:\n"
              << transformation_acording_to_pca_fixed * best_transformation_icp * transformation_acording_to_pca_moving.inverse() << std::endl;

    return 0;
}