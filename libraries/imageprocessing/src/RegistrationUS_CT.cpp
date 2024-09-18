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

/*
All the functions are static because they are local, thus we don't want to have their signature leaking from this translation unit
*/

/*Function to segment the region of interest of the input cutted image. Applies a gaussin gilter, then a laplacian. The histogram of the laplacian is used to create a region of interest using a threashold.
The treashold is defined as the value of the bin in which all the bins at the left contain the target number of samples (this target number is a percentage of the total number of samples).
A mask is created with this region of interest, and the original values of the input image inside the region of interest are returned in the smallest volume possible.*/
static itk::Image<float, 3>::Pointer apply_laplacian(itk::Image<float, 3>::Pointer input_image, float sigma, float cuttoff_histogram_percentage, std::string suffix, bool write_images)
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

static int modify_image_with_transform(Eigen::Matrix<double,4,4> transform,ImageType::Pointer image){
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

static auto get_image_transform(ImageType::Pointer image){
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

static int print_image_with_transform(ImageType::Pointer image,std::string image_path){
    using WriterType = itk::ImageFileWriter<ImageType>;
    auto writer = WriterType::New();
    writer->SetFileName(image_path);
    writer->SetInput(image);
    writer->Update();
    return 1;
};

static inline double rad2deg(double in){
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians*in;
}

static void writePointCloudToFile(const std::string& filename, const Eigen::Matrix<double, Eigen::Dynamic, 3>& points) {
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

int register_volumes(ImageType::Pointer pointer2inputfixedimage,ImageType::Pointer pointer2inputmovingimage)
{
    //Segmentation parameters
    float fixed_sigma = 4;
    float moving_sigma = 4;
    float fixed_histogram_percentage = 0.1;
    float moving_histogram_percentage = 0.1;
    bool write_segmentation_volumes = false;

    //Pointcloud downsampling percentage
    float downsampling_percentage = 0.01;

    using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using OutputImageType = itk::Image<PixelType, Dimension>;
    using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
    using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
    using WriterType = itk::ImageFileWriter<OutputImageType>;
    using FixedImageReaderType = itk::ImageFileReader<ImageType>;
    using MaskType = itk::ImageMaskSpatialObject<Dimension>;
    using MovingImageReaderType = itk::ImageFileReader<ImageType>;

    //Create pointers for each of the cutted volumes and the moving full volume (this last one is to write the results with the final transforms)
    

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
    auto transform = [](double alpha){
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
                               0.0         ,      -1.0        , 0.0 ,
                               0.0         ,      0.0        , 1.0 ;

        Eigen::Matrix<double,4,4> T_rotation_extra = Eigen::Matrix<double,4,4>::Identity();
        T_rotation_extra.block<3,3>(0,0) = poorly_constrained_direction*flipping_direction;
        return T_rotation_extra;
    };

    //Generate the initial conditions for the moving point cloud
    std::printf("\nGenerating initial guesses...\n");
    std::vector<Eigen::Matrix<double,4,4>> angles_regular;
    for(double angle = 0; angle < 360.0 ; angle+=10.0){
        angles_regular.push_back(transform(rad2deg(angle)));
        angles_regular.push_back(transform_flipped_principal_component(rad2deg(angle)));
    }

    //Align the principal axis of both volumes with the directions of the world frame and push their centroids to the origin.
    //This is done with the pointer to the registration volume so as to preserve the original volume 
    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage_registration);
    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage_registration);

    //Just like before we will extract a point cloud from both moving and fixed images but this time is from the ones that are already moved to the origin.
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
    }

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
    }

    //Execute paralelized icp registration for all the initial configs
    auto run_parameterized_icp_optimization = [&](){
        std::vector<std::tuple<double, Eigen::Matrix<double,4,4>>> full_runs_inner;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            size_t counter = 0;
            for (const auto &initial_config : angles_regular){
                curan::utilities::Job job{"solving icp",[&](){
                    auto solution = icp_registration(initial_config);
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs_inner.emplace_back(solution);
                        ++counter;
                        std::printf("%.0f %% %.3f\n",(counter/(double)angles_regular.size())*100,std::get<0>(solution));
                    }              
                }};
                pool->submit(job);
            } 
        }
        return full_runs_inner;
    };

    //Vector with the solitions from icp
    auto paralel_solutions = run_parameterized_icp_optimization();

    //Find the transformation with the lowest optimizer value (corresponds to the best alignment)
    Eigen::Matrix<double, 4, 4> best_transformation;
    auto min_element_iter = std::min_element(paralel_solutions.begin(), paralel_solutions.end(),
        [](const auto& a, const auto& b) {
            return std::get<0>(a) < std::get<0>(b);
        });

    if (min_element_iter != paralel_solutions.end()) {
        double min_value = std::get<0>(*min_element_iter);
        best_transformation = std::get<1>(*min_element_iter);
        
        //Output the minimum value and its corresponding transformation
        std::cout << "Minimum Value: " << min_value << std::endl;
        std::cout << "Corresponding Transformation Matrix: \n" << best_transformation << std::endl;

    }

    std::cout << "Final transform: \n" << T_origin_fixed*best_transformation.inverse()*T_origin_moving.inverse() << std::endl;

    //Write results
    modify_image_with_transform(best_transformation.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_icp.mha");

    modify_image_with_transform(T_origin_fixed.inverse()*Timage_origin_fixed,pointer2fixedimage);
    print_image_with_transform(pointer2fixedimage,"fixed_image_moved_to_origin.mha");

    modify_image_with_transform(T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_image_moved_to_origin.mha");

    modify_image_with_transform(Timage_origin_fixed*Timage_origin_fixed.inverse()*T_origin_fixed* best_transformation.inverse()*T_origin_moving.inverse()*Timage_origin_moving,pointer2movingimage);
    print_image_with_transform(pointer2movingimage,"moving_correct_in_fixed.mha");

    return 0;
}