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

using ImageReaderType = itk::ImageFileReader<ImageType>;

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

template <typename T>
void compute_itk_filter(T &filter)
{
    try
    {
        filter->Compute();
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

const double pi = std::atan(1) * 4;

/*Function to segment the region of interest of the input cutted image. Applies a gaussin gilter, then a laplacian. The histogram of the laplacian is used to create a region of interest using a threashold.
The treashold is defined as the value of the bin in which all the bins at the left contain the target number of samples (this target number is a percentage of the total number of samples).
A mask is created with this region of interest, and the original values of the input image inside the region of interest are returned in the smallest volume possible.*/
itk::Image<float, 3>::Pointer apply_laplacian(itk::Image<float, 3>::Pointer input_image, float sigma, float cuttoff_histogram_percentage, std::string suffix, bool write_images)
{
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
    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<ImageType>;
    using HistogramType = HistogramGeneratorType::HistogramType;

    // Rescale the input image between 0 and 1
    using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale_filter = RescaleFilterType::New();
    rescale_filter->SetInput(input_image);
    rescale_filter->SetOutputMinimum(0.0);
    rescale_filter->SetOutputMaximum(1.0);

    try
    {
        rescale_filter->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        // return 1;
    }

    ImageType::Pointer pointer2inputimage = rescale_filter->GetOutput();

    // Apply gaussian filter and write volume (optional)
    auto gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(pointer2inputimage);
    gaussianFilter->SetSigma(sigma);
    gaussianFilter->Update();

    auto writer = WriterType::New();
    if (write_images == true)
    {
        writer->SetInput(gaussianFilter->GetOutput());
        std::string output_name1 = "Gaussian_filtered_" + suffix + ".mha";
        writer->SetFileName(output_name1);
        try
        {
            writer->Update();
        }
        catch (const itk::ExceptionObject &err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            // return EXIT_FAILURE;
        }
    }

    // Apply laplacian filter to the output of the gaussian and write volume (optional)
    using FilterType10 = itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;
    auto laplacian = FilterType10::New();
    laplacian->SetNormalizeAcrossScale(true);
    laplacian->SetInput(gaussianFilter->GetOutput());
    laplacian->Update();

    if (write_images == true)
    {
        writer->SetInput(laplacian->GetOutput());
        std::string output_name2 = "Laplacian_" + suffix + ".mha";
        writer->SetFileName(output_name2);
        try
        {
            writer->Update();
        }
        catch (const itk::ExceptionObject &err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            // return EXIT_FAILURE;
        }
    }

    // Calculate minimum value of the laplacian
    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(laplacian->GetOutput());
    minMaxCalculator->Compute();
    PixelType minValue = minMaxCalculator->GetMinimum();

    // Construct histogram from the laplacian output
    auto histogramGenerator = HistogramGeneratorType::New();
    histogramGenerator->SetInput(laplacian->GetOutput());
    histogramGenerator->SetNumberOfBins(500);

    try
    {
        histogramGenerator->Compute();
    }
    catch (itk::ExceptionObject &error)
    {
        std::cerr << "Error computing histogram: " << error << std::endl;
        // return EXIT_FAILURE;
    }

    const HistogramType *histogram = histogramGenerator->GetOutput();

    // Total number of samples
    int total_frequency = 0;
    for (unsigned int i = 0; i < histogram->Size(); ++i)
    {
        total_frequency += histogram->GetFrequency(i);
    }

    // The treashold is defined as the value of the bin in which all the bins at the left contain the target number of samples (as a percentage of the total number)
    auto target_frequency = cuttoff_histogram_percentage * total_frequency;

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

    // Threshold the laplacian volume so as to contain only the voxels inside the threshold, and set the others to 0
    using ThresholdFilterType = itk::ThresholdImageFilter<ImageType>;
    ThresholdFilterType::Pointer thresholdFilter = ThresholdFilterType::New();
    thresholdFilter->SetInput(laplacian->GetOutput());
    thresholdFilter->ThresholdOutside(minValue, thresholdvalue); // 0.02 para a precious

    // thresholdFilter->ThresholdOutside(minMaxCalculator->GetMinimum(), thresholdvalue);
    thresholdFilter->SetOutsideValue(0);
    thresholdFilter->Update();

    // Calculate conected components from the laplacian
    using LabelType = unsigned short;
    using LabelImageType = itk::Image<LabelType, 3>;
    using ConnectedComponentFilterType = itk::ConnectedComponentImageFilter<ImageType, LabelImageType>;
    ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();
    connectedComponentFilter->SetInput(thresholdFilter->GetOutput());
    connectedComponentFilter->Update();

    // Label the components from largest to smallest
    using RelabelFilterType = itk::RelabelComponentImageFilter<LabelImageType, LabelImageType>;
    RelabelFilterType::Pointer relabelFilter = RelabelFilterType::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
    relabelFilter->Update();

    // std::cout << "Relabeling completed. Number of objects: " << relabelFilter->GetNumberOfObjects() << std::endl;

    // Threshold so as to keep only the 2 largest components
    using ThresholdFilterType2 = itk::ThresholdImageFilter<LabelImageType>;
    ThresholdFilterType2::Pointer thresholdFilter2 = ThresholdFilterType2::New();
    thresholdFilter2->SetInput(relabelFilter->GetOutput());
    thresholdFilter2->ThresholdOutside(1, 2);
    thresholdFilter2->SetOutsideValue(0);
    thresholdFilter2->Update();

    // Create mask from the largest components
    LabelImageType::Pointer largestComponentMask = thresholdFilter2->GetOutput();

    // Use a mask filter to get the original image information but only in the region of interest
    using MaskFilterType = itk::MaskImageFilter<ImageType, LabelImageType, ImageType>;
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    maskFilter->SetInput(pointer2inputimage);
    maskFilter->SetMaskImage(largestComponentMask);
    maskFilter->Update();

    // Create the smallest volume possible that contains all the required region
    using ConstIteratorType = itk::ImageRegionConstIterator<LabelImageType>;
    ConstIteratorType inputIt(thresholdFilter2->GetOutput(), thresholdFilter2->GetOutput()->GetRequestedRegion());

    std::pair<size_t, size_t> xlimits = {std::numeric_limits<size_t>::max(), 0};
    std::pair<size_t, size_t> ylimits = {std::numeric_limits<size_t>::max(), 0};
    std::pair<size_t, size_t> zlimits = {std::numeric_limits<size_t>::max(), 0};
    while (!inputIt.IsAtEnd())
    {
        if (inputIt.Get() > 0)
        {
            const auto &index = inputIt.GetIndex();
            if (xlimits.first > index[0])
            {
                xlimits.first = index[0];
            }
            if (xlimits.second < index[0])
            {
                xlimits.second = index[0];
            }
            if (ylimits.first > index[1])
            {
                ylimits.first = index[1];
            }
            if (ylimits.second < index[1])
            {
                ylimits.second = index[1];
            }
            if (zlimits.first > index[2])
            {
                zlimits.first = index[2];
            }
            if (zlimits.second < index[2])
            {
                zlimits.second = index[2];
            }
        }
        ++inputIt;
    }

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

    filter->SetExtractionRegion(desiredRegion);
    if (write_images == true)
    {
        writer->SetInput(filter->GetOutput());
        std::string output_name3 = "Segmented_volume_" + suffix + ".mha";
        writer->SetFileName(output_name3);
        update_ikt_filter(writer);
    }

    ImageType::Pointer segmented_volume = filter->GetOutput();
    return segmented_volume;
}

int modify_image_with_transform(Eigen::Matrix<double, 4, 4> transform, ImageType::Pointer image)
{
    itk::Point<double, 3> origin;
    itk::Matrix<double> direction;
    for (size_t row = 0; row < 3; ++row)
    {
        origin[row] = transform(row, 3);
        for (size_t col = 0; col < 3; ++col)
        {
            direction(row, col) = transform(row, col);
        }
    }
    image->SetOrigin(origin);
    image->SetDirection(direction);
    return 1;
};

auto get_image_transform(ImageType::Pointer image)
{
    Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Identity();
    itk::Point<double, 3> origin = image->GetOrigin();
    itk::Matrix<double> direction = image->GetDirection();
    for (size_t row = 0; row < 3; ++row)
    {
        transform(row, 3) = image->GetOrigin()[row];
        for (size_t col = 0; col < 3; ++col)
        {
            transform(row, col) = direction(row, col);
        }
    }
    return transform;
};

void print_image_with_transform(ImageType::Pointer image, std::string image_path)
{
    using WriterType = itk::ImageFileWriter<ImageType>;
    auto writer = WriterType::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    update_ikt_filter(writer);
    return;
};

inline double rad2deg(double in)
{
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians * in;
}

void writePointCloudToFile(const std::string &filename, const Eigen::Matrix<double, Eigen::Dynamic, 3> &points)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i < points.rows(); ++i)
    {
        file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << "\n";
    }
    file.close();
}

using MeshType = itk::Mesh<double>;

MeshType::Pointer recompute_and_simplify_mesh(MeshType::Pointer input_mesh)
{
    class MeshSimplierVisitor
    {
    public:
        using identifier_in_original_mesh = size_t;
        using identifier_in_post_processed_mesh = size_t;
        MeshType::Pointer mesh;
        using MeshSourceType = itk::AutomaticTopologyMeshSource<MeshType>;
        MeshSourceType::Pointer mesh_source;
        std::unordered_map<identifier_in_original_mesh, identifier_in_post_processed_mesh> identifiers;
        Eigen::Matrix<double, 3, 1> centroid;

        void set_required_data(MeshType::Pointer inmesh, Eigen::Matrix<double, 3, 1> incentroid)
        {
            mesh = inmesh;
            mesh_source = MeshSourceType::New();
            centroid = incentroid;
        }

        using TriangleType = itk::TriangleCell<MeshType::CellType>;
        void
        Visit(unsigned long cellId, TriangleType *t)
        {
            TriangleType::PointIdIterator pit = t->PointIdsBegin();
            TriangleType::PointIdIterator end = t->PointIdsEnd();
            Eigen::Matrix<double, 3, 3> points_in_cell;
            std::vector<identifier_in_original_mesh> identifiers_local;
            size_t col = 0;
            for (; pit != end; ++pit, ++col)
            {
                identifiers_local.emplace_back(*pit);
                auto point = mesh->GetPoint(*pit);
                points_in_cell(0, col) = point[0];
                points_in_cell(1, col) = point[1];
                points_in_cell(2, col) = point[2];
            }
            using IdentifierArrayType = MeshSourceType::IdentifierArrayType;

            // check if cell is towards center
            Eigen::Matrix<double, 3, 1> along_first_edge = points_in_cell.col(1) - points_in_cell.col(0);
            Eigen::Matrix<double, 3, 1> along_second_edge = points_in_cell.col(2) - points_in_cell.col(0);
            Eigen::Matrix<double, 3, 1> normal_to_cell = along_first_edge.cross(along_second_edge);

            normal_to_cell.normalize();

            Eigen::Matrix<double, 3, 1> center_of_face = points_in_cell.rowwise().mean();
            Eigen::Matrix<double, 3, 1> centroid_to_face_normalized_vector = center_of_face - centroid;
            centroid_to_face_normalized_vector.normalize();

            if (centroid_to_face_normalized_vector.transpose() * normal_to_cell > -0.23)
                return;

            MeshType::PointType p;
            MeshSourceType::IdentifierArrayType idArray(3);
            assert(identifiers_local.size() == points_in_cell.cols());
            size_t collum = 0;
            for (size_t collum = 0; collum < 3; ++collum)
            {
                auto search = identifiers.find(identifiers_local[collum]);
                if (search != identifiers.end())
                {
                    idArray[collum] = search->second;
                }
                else
                {
                    p[0] = points_in_cell(0, collum);
                    p[1] = points_in_cell(1, collum);
                    p[2] = points_in_cell(2, collum);
                    idArray[collum] = mesh_source->AddPoint(p);
                    identifiers.emplace(identifiers_local[collum], idArray[collum]);
                }
            }
            mesh_source->AddTriangle(idArray[0], idArray[1], idArray[2]);
        }

        MeshSimplierVisitor() = default;
        virtual ~MeshSimplierVisitor() = default;
    };

    Eigen::Matrix<double, 3, 1> centroid = Eigen::Matrix<double, 3, 1>::Zero();

    using PointsIterator = MeshType::PointsContainer::Iterator;
    for (PointsIterator pointIterator_fixed = input_mesh->GetPoints()->Begin(); pointIterator_fixed != input_mesh->GetPoints()->End(); ++pointIterator_fixed)
    {
        auto p = pointIterator_fixed->Value();
        Eigen::Matrix<double, 3, 1> point;
        point << p[0], p[1], p[2];
        centroid += point * (1.0 / input_mesh->GetNumberOfPoints());
    }
    using TriangleType = itk::TriangleCell<MeshType::CellType>;

    using TriangleVisitorInterfaceType =
        itk::CellInterfaceVisitorImplementation<MeshType::PixelType,
                                                MeshType::CellTraits,
                                                TriangleType,
                                                MeshSimplierVisitor>;
    auto triangleVisitor = TriangleVisitorInterfaceType::New();

    triangleVisitor->set_required_data(input_mesh, centroid);

    using CellMultiVisitorType = MeshType::CellType::MultiVisitor;
    auto multiVisitor = CellMultiVisitorType::New();
    multiVisitor->AddVisitor(triangleVisitor);
    input_mesh->Accept(multiVisitor);

    return triangleVisitor->mesh_source->GetOutput();
}

int main(int argc, char **argv)
{
    std::string path_fixed{"C:/Dev/NeuroNavigation/volumes/Stitched_US_2_sides.mha"};

    std::printf("\nReading input volumes...\n");
    auto fixedImageReader = ImageReaderType::New();
    fixedImageReader->SetFileName(path_fixed);
    try
    {
        fixedImageReader->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    ImageType::Pointer pointer2inputfixedimage = fixedImageReader->GetOutput();

    // Segmentation parameters
    float fixed_sigma = 4;
    float moving_sigma = 4;
    float fixed_histogram_percentage = 0.1;
    float moving_histogram_percentage = 0.1;
    bool write_segmentation_volumes = true;

    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;

    // Pointcloud downsampling percentage
    float downsampling_percentage = 0.01;

    // Rotation threashold between icp and mi solution. This value is the angle of the rotation matrix between the 2 solutions in axis angle representation
    const double rotation_threshold = 10.0; // in degrees

    // Preprocess the cutted volumes using laplacian and create pointers for them. These are the ones that will effectively be used with registration
    std::printf("\nPreprocessing input volumes...\n");
    auto pointer2fixedimage = apply_laplacian(pointer2inputfixedimage, fixed_sigma, fixed_histogram_percentage, "fixed", write_segmentation_volumes);
    print_image_with_transform(pointer2inputfixedimage, "laplaced_image.mha");
    // Create matrix to store direction and origin that come from the results of the PCA
    Eigen::Matrix<double, 4, 4> T_origin_fixed = Eigen::Matrix<double, 4, 4>::Identity();

    std::printf("\nExtracting fixed point cloud...\n");
    // For the fixed image

    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;

    // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale = RescaleType::New();
    rescale->SetInput(pointer2fixedimage);
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<MaskPixelType>::max());
    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    auto castfilter = CastFilterType::New();
    castfilter->SetInput(rescale->GetOutput());

    // Create a binary threshold that sets all the non zero voxels to 1. This means that the region of interest will have value 1.
    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType, MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->SetInsideValue(1);
    filter_threshold->SetLowerThreshold(1);
    filter_threshold->SetUpperThreshold(255);

    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(1); // 1 Because the region of interest has value 1.
    meshSource->SetInput(filter_threshold->GetOutput());
    update_ikt_filter(meshSource);

    MeshType::Pointer mesh = meshSource->GetOutput();

    using MeshSpatialObjectType = itk::MeshSpatialObject<MeshType>;
    auto myMeshSpatialObject = MeshSpatialObjectType::New();
    myMeshSpatialObject->SetMesh(mesh);
    myMeshSpatialObject->Update();
    myMeshSpatialObject->GetMesh();

    MeshType::Pointer simplified_mesh = recompute_and_simplify_mesh(mesh);

    {
        using WriterType = itk::MeshFileWriter<MeshType>;
        auto writer = WriterType::New();
        writer->SetFileName("fixed_point_cloud.obj");
        writer->SetInput(mesh);

        std::cout << "writing mesh ...\n";
        update_ikt_filter(writer);
    }

    {
        using WriterType = itk::MeshFileWriter<MeshType>;
        auto writer = WriterType::New();
        writer->SetFileName("fixed_point_cloud_removed_stuff.obj");
        writer->SetInput(simplified_mesh);

        std::cout << "writing mesh ...\n";
        update_ikt_filter(writer);
    }

    std::cout << "point cloud written\n";
    return 0;
}