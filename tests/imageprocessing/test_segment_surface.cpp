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
#include "itkHistogram.h"
#include "itkSampleToHistogramFilter.h"
#include "itkMaskImageFilter.h"
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkExtractImageFilter.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkStatisticsImageFilter.h"
#include "itkScalarImageToHistogramGenerator.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageRegionIteratorWithIndex.h"
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
#include "itkMeshFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkCommand.h"
#include "itkBinaryMask3DMeshSource.h"
#include <type_traits>
#include <nlohmann/json.hpp>
#include "utils/TheadPool.h"
#include "itkRegionOfInterestImageFilter.h"
#include <itkTransformGeometryImageFilter.h>
#include <iomanip>
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkNormalVariateGenerator.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkImageMaskSpatialObject.h"
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
#include "itkBinomialBlurImageFilter.h"

typedef itk::Mesh<double> MeshType;
typedef itk::BinaryMask3DMeshSource< ImageType,MeshType> MeshSourceType;

template <typename T>
void update_ikt_filter(T& filter)
{
    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject& err)
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

itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::Pointer convert_mesh(const itk::Mesh<double>::Pointer& mesh)
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
            Visit(unsigned long cellId, TriangleType* t)
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
            Visit(unsigned long cellId, TriangleType* t)
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
    Eigen::Matrix<double, 3, 1> centroid = Eigen::Matrix<double, 3, 1>::Zero();
    {
        itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::PointsContainerIterator p_it = converted_mesh->GetPoints()->Begin();
        double number_of_points = converted_mesh->GetNumberOfPoints();
        while (p_it != converted_mesh->GetPoints()->End())
        {
            centroid[0] += (1.0 / number_of_points) * p_it.Value()[0];
            centroid[1] += (1.0 / number_of_points) * p_it.Value()[1];
            centroid[2] += (1.0 / number_of_points) * p_it.Value()[2];
            ++p_it;
        }
    }

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
        point[0] = p_it.Value()[0] - centroid[0];
        point[1] = p_it.Value()[1] - centroid[1];
        point[2] = p_it.Value()[2] - centroid[2];
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

void write_point_set(const std::string& filename, itk::PointSet<double, 3>::Pointer pointset)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);
    for (itk::PointSet<double, 3>::PointsContainer::Iterator iterator = pointset->GetPoints()->Begin(); iterator < pointset->GetPoints()->End(); ++iterator)
        file << iterator->Value()[0] << " , " << iterator->Value()[1] << " , " << iterator->Value()[2] << "\n";
    file.close();
}

int main(int argc, char* argv[]){

    if (argc != 3)
    {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0];
        std::cerr << " <InputFileName> <Lower Threshold> <Upper Threshold>";
        std::cerr << std::endl;
        return EXIT_FAILURE;
    }

    auto lowerThreshold = static_cast<PixelType>(std::stoi(argv[1]));
    auto upperThreshold = static_cast<PixelType>(std::stoi(argv[2]));

    auto image_reader_fixed = itk::ImageFileReader<itk::Image<float, 3>>::New();
    image_reader_fixed->SetFileName(CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha");

    auto rescaler = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>::New();
    rescaler->SetInput(image_reader_fixed->GetOutput());
    rescaler->SetOutputMinimum(0.0);
    rescaler->SetOutputMaximum(255.0);

    using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<ImageType, ImageType>;
    auto threshold = BinaryThresholdFilterType::New();
    threshold->SetInput(rescaler->GetOutput());
    threshold->SetLowerThreshold(lowerThreshold);
    threshold->SetUpperThreshold(upperThreshold);
    threshold->SetOutsideValue(0);
    threshold->SetInsideValue(255.0);

    MeshSourceType::Pointer meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(255.0);
    meshSource->SetInput(threshold->GetOutput());

    using WriterType = itk::MeshFileWriter<MeshType>;
    auto writer = WriterType::New();
    writer->SetFileName(CURAN_COPIED_RESOURCE_PATH"/file.obj");
    writer->SetInput(meshSource->GetOutput());
    try
    {
        writer->Update();
    }
    catch (const itk::ExceptionObject& error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    auto point_container = prune_surface(meshSource->GetOutput());
    write_point_set(CURAN_COPIED_RESOURCE_PATH"/point_cloud.txt", point_container);
    return 0;
}