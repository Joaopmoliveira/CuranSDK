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

const double pi = std::atan(1) * 4;
using PixelType = float;
using RegistrationPixelType = PixelType;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

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

    //Rescale the input image between 0 and 1
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
        //return 1;
    }

    ImageType::Pointer pointer2inputimage = rescale_filter->GetOutput();

    //Apply gaussian filter and write volume (optional)
    auto gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(pointer2inputimage);
    gaussianFilter->SetSigma(sigma);
    gaussianFilter->Update();

    auto writer = WriterType::New();
    if(write_images == true){
        writer->SetInput(gaussianFilter->GetOutput());
        std::string output_name1 = "Gaussian_filtered_" + suffix + ".mha";
        writer->SetFileName(output_name1);
        try
        {
            writer->Update();
        }
        catch (const itk::ExceptionObject & err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            //return EXIT_FAILURE;
        }
    }

    //Apply laplacian filter to the output of the gaussian and write volume (optional)
    using FilterType10 = itk::LaplacianRecursiveGaussianImageFilter<ImageType, ImageType>;
    auto laplacian = FilterType10::New();
    laplacian->SetNormalizeAcrossScale(true);
    laplacian->SetInput(gaussianFilter->GetOutput());
    laplacian->Update();

    if(write_images == true){
        writer->SetInput(laplacian->GetOutput());
        std::string output_name2 = "Laplacian_" + suffix + ".mha";
        writer->SetFileName(output_name2);
        try
        {
            writer->Update();
        }
        catch (const itk::ExceptionObject & err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            //return EXIT_FAILURE;
        }
    }

    //Calculate minimum value of the laplacian
    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(laplacian->GetOutput());
    minMaxCalculator->Compute();
    PixelType minValue = minMaxCalculator->GetMinimum();

    //Construct histogram from the laplacian output
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
        //return EXIT_FAILURE;
    }

    const HistogramType * histogram = histogramGenerator->GetOutput();

    //Total number of samples
    int total_frequency = 0;
    for (unsigned int i = 0; i < histogram->Size(); ++i)
    {
        total_frequency += histogram->GetFrequency(i);
    }

    //The treashold is defined as the value of the bin in which all the bins at the left contain the target number of samples (as a percentage of the total number)
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

    //Threshold the laplacian volume so as to contain only the voxels inside the threshold, and set the others to 0
    using ThresholdFilterType = itk::ThresholdImageFilter<ImageType>;
    ThresholdFilterType::Pointer thresholdFilter = ThresholdFilterType::New();
    thresholdFilter->SetInput(laplacian->GetOutput());
    thresholdFilter->ThresholdOutside(minValue, thresholdvalue); //0.02 para a precious
    //thresholdFilter->ThresholdOutside(minMaxCalculator->GetMinimum(), thresholdvalue);
    thresholdFilter->SetOutsideValue(0);
    thresholdFilter->Update();

    //Calculate conected components from the laplacian
    using LabelType = unsigned short;
    using LabelImageType = itk::Image<LabelType, 3>;
    using ConnectedComponentFilterType = itk::ConnectedComponentImageFilter<ImageType, LabelImageType>;
    ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();
    connectedComponentFilter->SetInput(thresholdFilter->GetOutput());
    connectedComponentFilter->Update();

    //Label the components from largest to smallest
    using RelabelFilterType = itk::RelabelComponentImageFilter<LabelImageType, LabelImageType>;
    RelabelFilterType::Pointer relabelFilter = RelabelFilterType::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
    relabelFilter->Update();

    //std::cout << "Relabeling completed. Number of objects: " << relabelFilter->GetNumberOfObjects() << std::endl;

    //Threshold so as to keep only the 2 largest components 
    using ThresholdFilterType2 = itk::ThresholdImageFilter<LabelImageType>;
    ThresholdFilterType2::Pointer thresholdFilter2 = ThresholdFilterType2::New();
    thresholdFilter2->SetInput(relabelFilter->GetOutput());
    thresholdFilter2->ThresholdOutside(1, 2); 
    thresholdFilter2->SetOutsideValue(0);
    thresholdFilter2->Update();

    //Create mask from the largest components
    LabelImageType::Pointer largestComponentMask = thresholdFilter2->GetOutput();

    //Use a mask filter to get the original image information but only in the region of interest
    using MaskFilterType = itk::MaskImageFilter<ImageType, LabelImageType, ImageType>;
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    maskFilter->SetInput(pointer2inputimage);
    maskFilter->SetMaskImage(largestComponentMask);
    maskFilter->Update();

    //Create the smallest volume possible that contains all the required region
    using ConstIteratorType = itk::ImageRegionConstIterator<LabelImageType>;
    ConstIteratorType inputIt(thresholdFilter2->GetOutput(), thresholdFilter2->GetOutput()->GetRequestedRegion());
    
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
    filter->Update();
    if(write_images == true){
        writer->SetInput(filter->GetOutput());
        std::string output_name3 = "Segmented_volume_" + suffix + ".mha";
        writer->SetFileName(output_name3);
        try
        {
            writer->Update();
        }
        catch (const itk::ExceptionObject & err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            //return EXIT_FAILURE;
        }
    }

    ImageType::Pointer segmented_volume = filter->GetOutput();
    return segmented_volume;
}

int modify_image_with_transform(Eigen::Matrix<double,4,4> transform,ImageType::Pointer image){
    itk::Point<double,3> origin;
    itk::Matrix<double> direction;
    for(size_t row = 0; row < 3; ++row){
        origin[row] = transform(row,3);
        for(size_t col = 0; col < 3; ++col){
            direction(row,col) = transform(row,col);
        }
     }
    image->SetOrigin(origin);
    image->SetDirection(direction);
    return 1;
};

auto get_image_transform(ImageType::Pointer image){
    Eigen::Matrix<double,4,4> transform = Eigen::Matrix<double,4,4>::Identity();
    itk::Point<double,3> origin = image->GetOrigin();
    itk::Matrix<double> direction= image->GetDirection();
    for(size_t row = 0; row < 3; ++row){
        transform(row,3) = image->GetOrigin()[row];
        for(size_t col = 0; col < 3; ++col){
            transform(row,col) = direction(row,col);
        }
     }
    return transform;
};

int print_image_with_transform(ImageType::Pointer image,std::string image_path){
    using WriterType = itk::ImageFileWriter<ImageType>;
    auto writer = WriterType::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    writer->Update();
    return 1;
};

inline double rad2deg(double in){
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians*in;
}

void writePointCloudToFile(const std::string& filename, const Eigen::Matrix<double, Eigen::Dynamic, 3>& points) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i < points.rows(); ++i) {
        file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << "\n";
    }
    file.close();
}

Eigen::Matrix<double, Eigen::Dynamic, 3> downsample_points(const Eigen::Matrix<double, Eigen::Dynamic, 3>& points_in_matrix_form, double downsampling_percentage) {
    size_t total_points = points_in_matrix_form.rows();
    size_t reduced_points = static_cast<size_t>(total_points * downsampling_percentage);
    
    std::vector<size_t> indices(total_points);
    for (size_t i = 0; i < total_points; ++i) {
        indices[i] = i;
    }
    
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    indices.resize(reduced_points);
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> selected_points(reduced_points, 3);
    for (size_t i = 0; i < reduced_points; ++i) {
        selected_points.row(i) = points_in_matrix_form.row(indices[i]);
    }
    
    return selected_points;
}

/*This registration is very standard and almost equal to the itk examples. The main difference is that Eigen is used for transforms.*/
std::tuple<double,Eigen::Matrix<double,4,4>> icp_registration(Eigen::Matrix4d initial_config){
    constexpr unsigned int Dimension = 3;
    using PointSetType = itk::PointSet<float, Dimension>;
    
    auto fixedPointSet = PointSetType::New();
    auto movingPointSet = PointSetType::New();
    
    using PointType = PointSetType::PointType;
    using PointsContainer = PointSetType::PointsContainer;
    
    auto fixedPointContainer = PointsContainer::New();
    auto movingPointContainer = PointsContainer::New();
    
    PointType fixedPoint;
    PointType movingPoint;

    std::ifstream fixedFile;
    fixedFile.open("fixed_point_cloud.txt");
    if (fixedFile.fail())
    {
        std::cerr << "Error opening points file with name : " << std::endl;
        std::cerr << "fixed_point_cloud.txt" << std::endl;
    }
    
    unsigned int pointId = 0;
    fixedFile >> fixedPoint;
    while (!fixedFile.eof())
    {
        fixedPointContainer->InsertElement(pointId, fixedPoint);
        fixedFile >> fixedPoint;
        pointId++;
    }

    fixedPointSet->SetPoints(fixedPointContainer);
    
    // Read the file containing coordinates of moving points.
    std::ifstream movingFile;
    movingFile.open("moving_point_cloud.txt");
    if (movingFile.fail())
    {
        std::cerr << "Error opening points file with name : " << std::endl;
        std::cerr << "moving_point_cloud.txt" << std::endl;
    }
    
    pointId = 0;
    movingFile >> movingPoint;
    while (!movingFile.eof())
    {
        movingPointContainer->InsertElement(pointId, movingPoint);
        movingFile >> movingPoint;
        pointId++;
    }
    movingPointSet->SetPoints(movingPointContainer);

    using MetricType =
        itk::EuclideanDistancePointMetric<PointSetType, PointSetType>;
    auto metric = MetricType::New();
    using TransformType = itk::Euler3DTransform<double>;
    auto transform = TransformType::New();
    using OptimizerType = itk::LevenbergMarquardtOptimizer;
    auto optimizer = OptimizerType::New();
    optimizer->SetUseCostFunctionGradient(false);
    
    using RegistrationType =
        itk::PointSetToPointSetRegistrationMethod<PointSetType, PointSetType>;
    auto registration = RegistrationType::New();
    OptimizerType::ScalesType scales(transform->GetNumberOfParameters());
    constexpr double translationScale = 1000.0;
    constexpr double rotationScale = 1.0;       
    scales[0] = 1.0 / rotationScale;
    scales[1] = 1.0 / rotationScale;
    scales[2] = 1.0 / rotationScale;
    scales[3] = 1.0 / translationScale;
    scales[4] = 1.0 / translationScale;
    scales[5] = 1.0 / translationScale;
    
    unsigned long numberOfIterations = 2000;
    double        gradientTolerance = 1e-4; 
    double        valueTolerance = 1e-4;   
    double        epsilonFunction = 1e-5;  
    
    optimizer->SetScales(scales);
    optimizer->SetNumberOfIterations(numberOfIterations);
    optimizer->SetValueTolerance(valueTolerance);
    optimizer->SetGradientTolerance(gradientTolerance);
    optimizer->SetEpsilonFunction(epsilonFunction);

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    itk::Vector<double,3> origin;
    itk::Matrix<double> direction;

    for(size_t row = 0; row < 3; ++row){
        origin[row] = initial_config(row,3);
        for(size_t col = 0; col < 3; ++col){
            direction(row,col) = initial_config(row,col);
        }
    }

    initialTransform->SetMatrix(direction);
    initialTransform->SetTranslation(origin);

    registration->SetInitialTransformParameters(initialTransform->GetParameters());
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
    registration->SetTransform(transform);
    registration->SetFixedPointSet(fixedPointSet);
    registration->SetMovingPointSet(movingPointSet);

    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject & e)
    {
        std::cerr << e << std::endl;
    }
    
    Eigen::Matrix<double,4,4> final_transformation = Eigen::Matrix<double,4,4>::Identity();
    for(size_t row = 0; row < 3; ++row){
        final_transformation(row,3) = transform->GetOffset()[row];
        for(size_t col = 0; col < 3; ++col){
            final_transformation(row,col) = transform->GetMatrix()(row,col);
        }
    }
    
    return {optimizer->GetValue().two_norm(),final_transformation};
}

struct info_solve_registration
{
    using ImageRegistrationType = itk::Image<float, 3>;
    using MaskType = itk::ImageMaskSpatialObject<3>;
    ImageRegistrationType::Pointer fixed_image;
    ImageRegistrationType::Pointer moving_image;
    std::optional<MaskType::Pointer> fixed_image_mask;
    std::optional<MaskType::Pointer> moving_image_mask;
    const Eigen::Matrix<double,4,4> &initial_rotation;
};

constexpr size_t size_info = 1;

struct RegistrationParameters{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t,size_info> piramid_sizes;
    std::array<double,size_info> bluering_sizes;

    RegistrationParameters(size_t in_bin_numbers,
                           double in_relative_scales,
                           double in_learning_rate,
                           double in_sampling_percentage,
                           double in_relaxation_factor,
                           size_t in_convergence_window_size,
                           size_t in_optimization_iterations,
                           std::array<size_t,size_info> in_piramid_sizes,
                           std::array<double,size_info> in_bluering_sizes) : bin_numbers{in_bin_numbers},
                                                                                      relative_scales{in_relative_scales},
                                                                                      learning_rate{in_learning_rate},
                                                                                      sampling_percentage {in_sampling_percentage},
                                                                                      relaxation_factor{in_relaxation_factor},
                                                                                      convergence_window_size{in_convergence_window_size},
                                                                                      optimization_iterations{in_optimization_iterations},
                                                                                      piramid_sizes{in_piramid_sizes},
                                                                                      bluering_sizes{in_bluering_sizes}                                                    
    {

    }
};

std::tuple<double,Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>> solve_registration(const info_solve_registration &info_registration, const RegistrationParameters& parameters)
{
    using PixelType = float;
    using RegistrationPixelType = PixelType;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image<PixelType, Dimension>;
    using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageRegistrationType,double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType, ImageRegistrationType>;
    using RegistrationType = itk::ImageRegistrationMethodv4<ImageRegistrationType, ImageRegistrationType, TransformType>;

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    metric->SetNumberOfHistogramBins(parameters.bin_numbers);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);

    if(info_registration.fixed_image_mask)
        metric->SetFixedImageMask(*info_registration.fixed_image_mask);
    if(info_registration.moving_image_mask)
        metric->SetMovingImageMask(*info_registration.moving_image_mask);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    itk::Vector<double,3> origin;
    itk::Matrix<double> direction;
    //std::cout << info_registration.initial_rotation << std::endl;

    for(size_t row = 0; row < 3; ++row){
        origin[row] = info_registration.initial_rotation(row,3);
        for(size_t col = 0; col < 3; ++col){
            direction(row,col) = info_registration.initial_rotation(row,col);
        }
    }

    //sanity check 
    if(!(info_registration.initial_rotation.block<3,3>(0,0).transpose()*info_registration.initial_rotation.block<3,3>(0,0)).isDiagonal()){
        std::cout << "failure to initialize rotation matrix...\n";
        std::cout << "values are: \n";
        std::cout << info_registration.initial_rotation.block<3,3>(0,0) << std::endl;
        std::cout << "the multiplication with itself is:";
        std::cout << info_registration.initial_rotation.block<3,3>(0,0).transpose()*info_registration.initial_rotation.block<3,3>(0,0) << std::endl;
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

    std::srand(std::time(nullptr));
    
    optimizer->SetLearningRate(parameters.learning_rate);
    optimizer->SetMinimumStepLength(0.0001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{parameters.convergence_window_size};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(parameters.relaxation_factor);

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(size_info);
    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(size_info);

    for(size_t i = 0; i < size_info; ++i){
        shrinkFactorsPerLevel[i] = parameters.piramid_sizes[i];
        smoothingSigmasPerLevel[i] = parameters.bluering_sizes[i];
    }

    registration->SetNumberOfLevels(size_info);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);
    RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::REGULAR;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(parameters.sampling_percentage);
    registration->MetricSamplingReinitializeSeed(std::rand());
    registration->SetInPlace(false);

    try{
        registration->Update();
    } catch (const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return {100.0, Eigen::Matrix<double,4,4>::Identity(),info_registration.initial_rotation};
    }
    TransformType::Pointer final_registration = registration->GetModifiableTransform();
    Eigen::Matrix<double,4,4> final_transformation = Eigen::Matrix<double,4,4>::Identity();
    for(size_t row = 0; row < 3; ++row){
        final_transformation(row,3) = final_registration->GetOffset()[row];
        for(size_t col = 0; col < 3; ++col){
            final_transformation(row,col) = final_registration->GetMatrix()(row,col);
        }
    }
    return {optimizer->GetValue(), final_transformation,info_registration.initial_rotation};
}

int main(int argc, char **argv)
{
    if(argc!=5){
        if(argc>5 || argc == 1){
            std::cout << "To run the executable you must provide four arguments:\n"
                      << "First parameter - Cuted volume (fixed)\n"
                      << "Second parameter - Cuted volume (moving)\n"
                      << "Third parameter - Full moving volume\n";
                      return 1;
        }
        if(argc == 2){
            std::cout << "To run the executable you must provide four arguments:\n "
                      << "First parameter - " << std::string(argv[1]) << "\n"
                      << "Second parameter - Cuted volume (moving)\n"
                      << "Third parameter - Full moving volume\n";
                      return 1;
        }
        if(argc == 3){
            std::cout << "To run the executable you must provide four arguments:\n "
                      << "First parameter - " << std::string(argv[1]) << "\n"
                      << "Second parameter - "<< std::string(argv[2]) << "\n"
                      << "Third parameter - Full moving volume\n";
                      return 1;
        }
    }   
        
    //Segmentation parameters
    float fixed_sigma = 4;
    float moving_sigma = 4;
    float fixed_histogram_percentage = 0.1;
    float moving_histogram_percentage = 0.1;
    bool write_segmentation_volumes = false;

    //Pointcloud downsampling percentage
    float downsampling_percentage = 0.01;

    using PixelType = float;
    using RegistrationPixelType = PixelType;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image<PixelType, Dimension>;
    using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using OutputImageType = itk::Image<PixelType, Dimension>;
    using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
    using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
    using WriterType = itk::ImageFileWriter<OutputImageType>;
    using FixedImageReaderType = itk::ImageFileReader<ImageType>;
    using MaskType = itk::ImageMaskSpatialObject<Dimension>;
    using MovingImageReaderType = itk::ImageFileReader<ImageType>;

    try{
    //Set readers for each of the images from the input arguments
    //fixedImageReader and movingImageReader are the cutted input volumes (with only the region of interest for double sided registration of the temporals)
    //movingFullImageReader corresponds to the original US volumes from which the cuts were obtained
    std::printf("\nReading input volumes...\n");
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();
    auto movingFullImageReader = MovingImageReaderType::New();

    std::string dirName{argv[1]};
    fixedImageReader->SetFileName(dirName);

    std::string dirName2{argv[2]};
    movingImageReader->SetFileName(dirName2);

    std::string dirName3{argv[3]};
    movingFullImageReader->SetFileName(dirName3);

    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
        movingFullImageReader->Update();
    }
    catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    //Create pointers for each of the cutted volumes and the moving full volume (this last one is to write the results with the final transforms)
    ImageType::Pointer pointer2inputfixedimage = fixedImageReader->GetOutput();
    ImageType::Pointer pointer2inputmovingimage = movingImageReader->GetOutput();
    ImageType::Pointer pointer2fullmovingimage = movingFullImageReader->GetOutput();

    //Preprocess the cutted volumes using laplacian and create pointers for them. These are the ones that will effectively be used with registration
    std::printf("\nPreprocessing input volumes...\n");
    auto pointer2fixedimage = apply_laplacian(pointer2inputfixedimage, fixed_sigma, fixed_histogram_percentage, "fixed", write_segmentation_volumes);
    auto pointer2movingimage = apply_laplacian(pointer2inputmovingimage, moving_sigma, moving_histogram_percentage, "moving", write_segmentation_volumes);

    //Create matrix to store direction and origin that come from the results of the PCA
    Eigen::Matrix<double,4,4> T_origin_fixed = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,4,4> T_origin_moving = Eigen::Matrix<double,4,4>::Identity();

    //These pointers will be used later for registration
    ImageRegistrationType::Pointer pointer2fixedimage_registration;
    ImageRegistrationType::Pointer pointer2movingimage_registration;

    std::printf("\nExtracting fixed point cloud...\n");
    //For the fixed image 
    {
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;

    //Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale = RescaleType::New();
    rescale->SetInput(pointer2fixedimage);
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<MaskPixelType>::max());
    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    auto castfilter = CastFilterType::New();
    castfilter->SetInput(rescale->GetOutput());

    //Create a binary threshold that sets all the non zero voxels to 1. This means that the region of interest will have value 1. 
    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->SetInsideValue(1);   
    filter_threshold->SetLowerThreshold(1);
    filter_threshold->SetUpperThreshold(255);

    //Exctract a mesh from the region of interest 
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(1); //1 Because the region of interest has value 1. 
    meshSource->SetInput(filter_threshold->GetOutput());
    try{
        meshSource->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    auto mesh = meshSource->GetOutput();

    //This pointer will be used later for registration
    pointer2fixedimage_registration = rescale->GetOutput();

    //Convert the points to use Eigen
    Eigen::Matrix<double,Eigen::Dynamic,3> points_in_matrix_form = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    PointsIterator end = mesh->GetPoints()->End();
    size_t index = 0;
    while (pointIterator != end)
    {
        auto p = pointIterator->Value();
        points_in_matrix_form(index,0) = p[0];
        points_in_matrix_form(index,1) = p[1];
        points_in_matrix_form(index,2) = p[2];
        ++pointIterator;
        ++index;
    }

    //PCA 
    std::printf("\nPerforming PCA to fixed point cloud...\n");
    Eigen::Matrix<double,3,1> center_of_fixed_image = points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double,1,3> to_subtract = center_of_fixed_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;    
    Eigen::Matrix<double,3,3> covariance_of_fixed_surface = points_in_matrix_form.transpose()*points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd( covariance_of_fixed_surface, Eigen::ComputeFullV | Eigen::ComputeFullU );
    assert(svd.computeU());
    auto rotation_of_fixed_image = svd.matrixU();
    rotation_of_fixed_image.col(2) = rotation_of_fixed_image.col(0).cross(rotation_of_fixed_image.col(1));
    T_origin_fixed.block<3,3>(0,0) = rotation_of_fixed_image;
    T_origin_fixed.block(0,3,3,1) = center_of_fixed_image;
    }

    //For the moving image
    std::printf("\nExtracting moving point cloud...\n");
    {
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;

    //Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale = RescaleType::New();
    rescale->SetInput(pointer2movingimage);
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<MaskPixelType>::max());
    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    auto castfilter = CastFilterType::New();
    castfilter->SetInput(rescale->GetOutput());

    //Create a binary threshold that sets all the non zero voxels to 1. This means that the region of interest will have value 1. 
    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    FilterTypeThreshold::Pointer filter_threshold = FilterTypeThreshold::New();
    filter_threshold->SetInput(castfilter->GetOutput());
    filter_threshold->SetOutsideValue(0);
    filter_threshold->SetInsideValue(1);   
    filter_threshold->SetLowerThreshold(1);
    filter_threshold->SetUpperThreshold(255);

    //Exctract a mesh from the region of interest 
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto meshSource = MeshSourceType::New();
    meshSource->SetObjectValue(1); //1 Because the region of interest has value 1. 
    meshSource->SetInput(filter_threshold->GetOutput());
    try{
        meshSource->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    auto mesh = meshSource->GetOutput();

    //This pointer will be used later for registration
    pointer2movingimage_registration = rescale->GetOutput();

    //Convert the points to use Eigen
    Eigen::Matrix<double,Eigen::Dynamic,3> points_in_matrix_form = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator = mesh->GetPoints()->Begin();
    PointsIterator end = mesh->GetPoints()->End();
    size_t index = 0;
    while (pointIterator != end)
    {
        auto p = pointIterator->Value();
        points_in_matrix_form(index,0) = p[0];
        points_in_matrix_form(index,1) = p[1];
        points_in_matrix_form(index,2) = p[2];
        ++pointIterator;
        ++index;
    }

    //PCA
    std::printf("\nPerforming PCA to moving point cloud...\n");
    Eigen::Matrix<double,3,1> center_of_moving_image = points_in_matrix_form.colwise().mean().transpose();
    Eigen::Matrix<double,1,3> to_subtract = center_of_moving_image.transpose();
    points_in_matrix_form.rowwise() -= to_subtract;    
    Eigen::Matrix<double,3,3> covariance_of_moving_surface = points_in_matrix_form.transpose()*points_in_matrix_form;
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd( covariance_of_moving_surface, Eigen::ComputeFullV | Eigen::ComputeFullU );
    assert(svd.computeU());
    auto rotation_of_moving_image = svd.matrixU();
    rotation_of_moving_image.col(2) = rotation_of_moving_image.col(0).cross(rotation_of_moving_image.col(1));
    T_origin_moving.block<3,3>(0,0) = rotation_of_moving_image;
    T_origin_moving.block(0,3,3,1) = center_of_moving_image;
    }

    /*We will rotate the moving point cloud around the first principal axis which will give us the various strating conditions for icp registration*/

    //Get the orientation and origin of fixed and moving images in the world frame
    Eigen::Matrix<double,4,4> Timage_origin_fixed = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,4,4> Timage_origin_moving = Eigen::Matrix<double,4,4>::Identity();
    for(size_t row = 0; row < 3; ++row){
        Timage_origin_fixed(row,3) = pointer2fixedimage->GetOrigin()[row];
        Timage_origin_moving(row,3) = pointer2movingimage->GetOrigin()[row];
        for(size_t col = 0; col < 3; ++col){
            Timage_origin_fixed(row,col) = pointer2fixedimage->GetDirection()(row,col);
            Timage_origin_moving(row,col) = pointer2movingimage->GetDirection()(row,col);
        }
    }

    //Lamda to get a rotation matrix from a given angle
    auto transform_x = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction <<     1.0 ,       0.0       ,       0.0        ,
                                            0.0 , std::cos(alpha) , -std::sin(alpha) ,
                                            0.0 , std::sin(alpha) ,  std::cos(alpha) ;
        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction;
        return T_rotation_extra;
    };

    //Lamda to get a rotation matrix from a given angle but with a flipped principal direction
    auto transform_flipped_principal_component = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction <<     1.0 ,       0.0       ,       0.0        ,
                                            0.0 , std::cos(alpha) , -std::sin(alpha) ,
                                            0.0 , std::sin(alpha) ,  std::cos(alpha) ;

        Eigen::Matrix<double,3,3> flipping_direction;
        flipping_direction << -1.0         ,      0.0        , 0.0 ,
                               0.0         ,     -1.0        , 0.0 ,
                               0.0         ,      0.0        , 1.0 ;

        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction*flipping_direction;
        return T_rotation_extra;
    };

    auto transform_y = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha), 0.0, std::sin(alpha),
                                          0.0,           1.0,           0.0,
                                       -std::sin(alpha), 0.0, std::cos(alpha);
        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction;
        return T_rotation_extra;
    };

    auto transform_z = [](double alpha){
        Eigen::Matrix<double,3,3> poorly_constrained_direction;
        poorly_constrained_direction << std::cos(alpha), -std::sin(alpha), 0.0,
                                        std::sin(alpha),  std::cos(alpha), 0.0,
                                              0.0,             0.0,        1.0;
        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction;
        return T_rotation_extra;
    };

    //Lambda to create a translation matrix
    auto transform_translation = [](double tx, double ty, double tz) {
        Eigen::Matrix<double, 4, 4> T_translation = Eigen::Matrix<double, 4, 4>::Identity();
        T_translation(0, 3) = tx;
        T_translation(1, 3) = ty;
        T_translation(2, 3) = tz;
        return T_translation;
    };

    //Generate the initial conditions for the moving point cloud
    std::printf("\nGenerating initial guesses for ICP...\n");
    std::vector<Eigen::Matrix<double,4,4>> initial_guesses_icp;
    for(double angle = 0; angle < 360.0 ; angle+=10.0){
        initial_guesses_icp.push_back(transform_x(rad2deg(angle)));
        initial_guesses_icp.push_back(transform_flipped_principal_component(rad2deg(angle)));
    }

    //Align the principal axis of both volumes with the directions of the world frame and push their centroids to the origin.
    //This is done with the pointer to the registration volume so as to preserve the original volume 
    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage_registration);
    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage_registration);

    //Just like before we will extract a point cloud from both moving and fixed images but this time is from the ones that are already moved to the origin.
    
    auto fixedSpatialObjectMask = MaskType::New();
    //For fixed
    {
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;

    auto castfilter_fixed = CastFilterType::New();
    castfilter_fixed->SetInput(pointer2fixedimage_registration);
    auto filter_threshold_fixed = FilterTypeThreshold::New();
    filter_threshold_fixed->SetInput(castfilter_fixed->GetOutput());
    filter_threshold_fixed->SetOutsideValue(0);
    filter_threshold_fixed->SetInsideValue(1);   
    filter_threshold_fixed->SetLowerThreshold(1);
    filter_threshold_fixed->SetUpperThreshold(255);
    filter_threshold_fixed->Update();

    auto meshSource_fixed = MeshSourceType::New();
    meshSource_fixed->SetObjectValue(1);
    meshSource_fixed->SetInput(filter_threshold_fixed->GetOutput());
    try{
        meshSource_fixed->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    auto mesh_fixed = meshSource_fixed->GetOutput();
    Eigen::Matrix<double,Eigen::Dynamic,3> fixed_points = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh_fixed->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator_fixed = mesh_fixed->GetPoints()->Begin();
    PointsIterator end_fixed = mesh_fixed->GetPoints()->End();
    size_t index = 0;
    while (pointIterator_fixed != end_fixed)
    {
        auto p = pointIterator_fixed->Value();
        fixed_points(index,0) = p[0];
        fixed_points(index,1) = p[1];
        fixed_points(index,2) = p[2];
        ++pointIterator_fixed;
        ++index;
    }

    //Downsample and write pointcloud
    auto downsampled_fixed_points = downsample_points(fixed_points, downsampling_percentage);
    writePointCloudToFile("fixed_point_cloud.txt", downsampled_fixed_points);

    fixedSpatialObjectMask->SetImage(filter_threshold_fixed->GetOutput());

    try{
        fixedSpatialObjectMask->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }
    }

    auto movingSpatialObjectMask = MaskType::New();
    //For moving
    {
    using MaskPixelType = unsigned char;
    using MaskImageType = itk::Image<MaskPixelType, Dimension>;
    using CastFilterType = itk::CastImageFilter<ImageType, MaskImageType>;
    using FilterTypeThreshold = itk::BinaryThresholdImageFilter<MaskImageType,MaskImageType>;
    using MeshType =  itk::Mesh<double>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<MaskImageType, MeshType>;
    auto castfilter_moving = CastFilterType::New();
    castfilter_moving->SetInput(pointer2movingimage_registration);
    auto filter_threshold_moving = FilterTypeThreshold::New();
    filter_threshold_moving->SetInput(castfilter_moving->GetOutput());
    filter_threshold_moving->SetOutsideValue(0);
    filter_threshold_moving->SetInsideValue(1);   
    filter_threshold_moving->SetLowerThreshold(1);
    filter_threshold_moving->SetUpperThreshold(255);
    filter_threshold_moving->Update();

    auto meshSource_moving = MeshSourceType::New();
    meshSource_moving->SetObjectValue(1);
    meshSource_moving->SetInput(filter_threshold_moving->GetOutput());
    try{
        meshSource_moving->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    auto mesh_moving = meshSource_moving->GetOutput();
    Eigen::Matrix<double,Eigen::Dynamic,3> moving_points = Eigen::Matrix<double,Eigen::Dynamic,3>::Zero(mesh_moving->GetNumberOfPoints(),3);
    using PointsIterator = MeshType::PointsContainer::Iterator;
    PointsIterator pointIterator_moving = mesh_moving->GetPoints()->Begin();
    PointsIterator end_moving = mesh_moving->GetPoints()->End();
    size_t index2 = 0;
    while (pointIterator_moving != end_moving)
    {
        auto p = pointIterator_moving->Value();
        moving_points(index2,0) = p[0];
        moving_points(index2,1) = p[1];
        moving_points(index2,2) = p[2];
        ++pointIterator_moving;
        ++index2;
    }

    //Downsample and write pointcloud
    auto downsampled_moving_points = downsample_points(moving_points, downsampling_percentage);
    writePointCloudToFile("moving_point_cloud.txt", downsampled_moving_points);

    movingSpatialObjectMask->SetImage(filter_threshold_moving->GetOutput());

    try{
        movingSpatialObjectMask->Update();
    }catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }
    }

    //Execute paralelized icp registration for all the initial configs
    std::printf("\nPre-alignement using ICP...\n");
    auto run_parameterized_icp_optimization = [&](){
        std::vector<std::tuple<double, Eigen::Matrix<double,4,4>>> full_runs_inner;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            size_t counter = 0;
            for (const auto &initial_config : initial_guesses_icp){
                curan::utilities::Job job{"solving icp",[&](){
                    auto solution = icp_registration(initial_config);
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs_inner.emplace_back(solution);
                        ++counter;
                        std::printf("%.0f %% %.3f\n",(counter/(double)initial_guesses_icp.size())*100,std::get<0>(solution));
                    }              
                }};
                pool->submit(job);
            } 
        }
        return full_runs_inner;
    };

    //Vector with the solitions from icp
    auto paralel_solutions_icp = run_parameterized_icp_optimization();

    //Find the transformation with the lowest optimizer value (corresponds to the best alignment)
    Eigen::Matrix<double, 4, 4> best_transformation_icp;
    auto min_element_iter = std::min_element(paralel_solutions_icp.begin(), paralel_solutions_icp.end(),
        [](const auto& a, const auto& b) {
            return std::get<0>(a) < std::get<0>(b);
        });

    if (min_element_iter != paralel_solutions_icp.end()) {
        double min_value = std::get<0>(*min_element_iter);
        best_transformation_icp = std::get<1>(*min_element_iter);
        
        //Output the minimum value and its corresponding transformation
        std::cout << "Minimum Value: " << min_value << std::endl;
        std::cout << "Corresponding Transformation Matrix (ICP): \n" << best_transformation_icp << std::endl;

    }

    std::ofstream myfile;
    myfile.open("results_of_fullscale_optimization.csv");
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    //Optimizer parameters
    constexpr size_t local_permut = 1;
    std::array<size_t,local_permut> bin_numbers{50};
    std::array<double,local_permut> percentage_numbers{1};
    std::array<double,local_permut> relative_scales{1000.0};
    std::array<double,local_permut> learning_rate{0.1};
    std::array<double,local_permut> relaxation_factor{0.7};
    std::array<size_t,local_permut> optimization_iterations{400};
    std::array<size_t,local_permut> convergence_window_size{30};
    std::array<std::array<size_t,size_info>,local_permut> piramid_sizes{{{1}}};
    std::array<std::array<double,size_info>,local_permut> bluering_sizes{{{0}}};

    constexpr size_t total_permutations = bin_numbers.size()*percentage_numbers.size()*relative_scales.size()*learning_rate.size()*relaxation_factor.size()*convergence_window_size.size()*piramid_sizes.size()*bluering_sizes.size();
    std::vector<std::tuple<double,Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>>> full_runs;
    
    std::printf("\nGenerating initial guesses for MI...\n");
    std::vector<Eigen::Matrix<double,4,4>> initial_guesses_mi;

    //Currentely just rotation and translation on x are being applied (these were found to be the most critical)
    for (double angle_x = -5; angle_x <= 5; angle_x += 1) {
        for (double tx = -5; tx <= 5; tx += 5) {
            double angle_y = 0;
            double angle_z = 0;
            double ty = 0;
            double tz = 0;
            Eigen::Matrix<double, 4, 4> rotation_x = transform_x(rad2deg(angle_x));
            Eigen::Matrix<double, 4, 4> rotation_y = transform_y(rad2deg(angle_y));
            Eigen::Matrix<double, 4, 4> rotation_z = transform_z(rad2deg(angle_z));
            Eigen::Matrix<double, 4, 4> translation = transform_translation(tx, ty, tz);
            Eigen::Matrix<double, 4, 4> combined_transform = translation * rotation_z * rotation_y * rotation_x * best_transformation_icp;
            initial_guesses_mi.push_back(combined_transform);
        }
    }

    //Execute paralelized MI registration for the more aproximate initial configs
    std::printf("\nResgistering using MI...\n");
    auto run_parameterized_optimization = [&](size_t bins, size_t iters, double percentage, double relative_scales,double learning_rate, double relaxation_factor,size_t window_size, auto piramid_sizes, auto bluering_sizes){
        std::vector<std::tuple<double, Eigen::Matrix<double,4,4>,Eigen::Matrix<double,4,4>>> full_runs_inner;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            size_t counter = 0;
            for (const auto &initial_config : initial_guesses_mi){
                curan::utilities::Job job{"solving registration",[&](){
                    //auto solution = solve_registration(info_solve_registration{pointer2fixedimage_registration, pointer2movingimage_registration,fixedSpatialObjectMask,movingSpatialObjectMask,min_transformation},RegistrationParameters{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    auto solution = solve_registration(info_solve_registration{pointer2fixedimage_registration, pointer2movingimage_registration,fixedSpatialObjectMask,movingSpatialObjectMask,initial_config},RegistrationParameters{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs_inner.emplace_back(solution);
                        ++counter;
                        std::printf("%.0f %% %.3f\n",(counter/(double)initial_guesses_mi.size())*100,std::get<0>(solution));
                    }              
                }};
                pool->submit(job);
            } 
        }
        return full_runs_inner;
    };

    size_t total_runs = 0;
    for(const auto& bin_n : bin_numbers)
        for(const auto& percent_n : percentage_numbers)
            for(const auto& rel_scale : relative_scales)
                for(const auto& learn_rate : learning_rate)
                    for(const auto& relax_factor : relaxation_factor)
                        for(const auto& wind_size : convergence_window_size)
                            for(const auto& pira_size : piramid_sizes)
                                for(const auto& iters : optimization_iterations)
                                    for(const auto& blur_size : bluering_sizes){
                                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                        auto paralel_solutions = run_parameterized_optimization(bin_n,iters,percent_n,rel_scale,learn_rate,relax_factor,wind_size,pira_size,blur_size);
                                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                        for(auto&& run : paralel_solutions){
                                            myfile << total_runs << "," << bin_n << "," << percent_n << "," << rel_scale << "," << learn_rate << "," << relax_factor << "," << wind_size << ", {";
                                            for(const auto& val : pira_size)
                                                myfile << val << ";";
                                            myfile << "}, {";
                                            for(const auto& val : blur_size)
                                                myfile << val << ";"; 
                                            myfile <<"}," << std::get<0>(run) << "," << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
                                        }
                                        ++total_runs;
                                        full_runs.insert(std::end(full_runs),std::begin(paralel_solutions),std::end(paralel_solutions));          
                                    } 

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = std::numeric_limits<double>::max();
    for (const auto &possible_best_solution : full_runs){
        if (minimum_val > std::get<0>(possible_best_solution)){
            minimum_index = current_index;
            minimum_val = std::get<0>(possible_best_solution);
        }
        ++current_index;
    }

    std::printf("Choosen cost: %.3f\n",minimum_val);
    
    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    std::cout << "Best estimated transform out of " << full_runs.size() << " is: \n" << std::get<1>(full_runs[minimum_index]) << std::endl ;

    std::cout << "Final transform: \n" << T_origin_fixed*finalTransform.inverse()*T_origin_moving.inverse() << std::endl;

    //Write results
    std::printf("\nWriting results...\n");
    auto original_full_moving_transform = get_image_transform(pointer2fullmovingimage);
    modify_image_with_transform(best_transformation_icp.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_icp.mha");

    modify_image_with_transform(T_origin_fixed*best_transformation_icp.inverse()*T_origin_moving.inverse()*get_image_transform(pointer2fullmovingimage),pointer2fullmovingimage);
    print_image_with_transform(pointer2fullmovingimage,"full_moving_correct_in_fixed_icp.mha");

    modify_image_with_transform(finalTransform.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_mi.mha");
   
    modify_image_with_transform(T_origin_fixed*finalTransform.inverse()*T_origin_moving.inverse()*original_full_moving_transform,pointer2fullmovingimage);
    print_image_with_transform(pointer2fullmovingimage,"full_moving_correct_in_fixed_mi.mha");

    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage);
    print_image_with_transform(pointer2fixedimage,"fixed_image_moved_to_origin.mha");

    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_image_moved_to_origin.mha");

    modify_image_with_transform(Timage_origin_fixed*Timage_origin_fixed.inverse()*T_origin_fixed* finalTransform.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_in_fixed_mi.mha");

    modify_image_with_transform(Timage_origin_fixed*Timage_origin_fixed.inverse()*T_origin_fixed* best_transformation_icp.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_in_fixed_icp.mha");

    modify_image_with_transform(std::get<2>(full_runs[minimum_index]).inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_initial_guess.mha");

    return 0;
}
    catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return 1;
    } catch(...){
        std::cout << "Some exception was thrown.\n";
        return 1;
    }
    return 0;
}