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
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include <optional>
#include <nlohmann/json.hpp>
#include <iostream>
#include "utils/TheadPool.h"
#include "itkRegionOfInterestImageFilter.h"
#include <fstream>
#include "itkImage.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkBinaryMorphologicalClosingImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkMesh.h"
#include "itkPointSet.h"
#include "itkCovariantVector.h"
#include "itkImageMomentsCalculator.h"
#include "itkMatrix.h"
#include "itkVersorRigid3DTransform.h"
#include "itkImage.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkBinaryMorphologicalClosingImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkRelabelComponentImageFilter.h"
#include "itkBinaryMorphologicalOpeningImageFilter.h"
#include "itkBinaryErodeImageFilter.h"
#include "itkBinaryDilateImageFilter.h"
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_symmetric_eigensystem.h>
#include <optional>
#include <fstream>
#include <itkResampleImageFilter.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkTransform.h>
#include <itkVersorRigid3DTransform.h>
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkBinaryContourImageFilter.h"
#include <Eigen/Dense>
#include <vector>


const double pi = std::atan(1) * 4;

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::VersorRigid3DTransform<double>;
using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;
using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<PixelType, Dimension>;
using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
using WriterType = itk::ImageFileWriter<OutputImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;
using MovingImageReaderType = itk::ImageFileReader<ImageType>;
using InterpolatorType = itk::LinearInterpolateImageFunction<
            ImageType,
            double>;


ImageType::Pointer manipulate_input_image(ImageType::Pointer image)
{
    using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(image);
    duplicator->Update();

    ImageType::Pointer new_image = duplicator->GetOutput();
    auto origin = image->GetOrigin();

    auto new_origin = origin;
    new_origin[0] += 1111.0;
    new_origin[1] += 1111.0;
    new_origin[2] += 1111.0;

    new_image->SetOrigin(new_origin);

    auto direction = image->GetDirection();

    double euler_vector[3] = {1.1, -1.1, 1.1};

    double t1 = std::cos(euler_vector[2]);
    double t2 = std::sin(euler_vector[2]);
    double t3 = std::cos(euler_vector[1]);
    double t4 = std::sin(euler_vector[1]);
    double t5 = std::cos(euler_vector[0]);
    double t6 = std::sin(euler_vector[0]);

    direction(0, 0) = t1 * t3;
    direction(0, 1) = t1 * t4 * t6 - t2 * t5;
    direction(0, 2) = t2 * t6 + t1 * t4 * t5;

    direction(1, 0) = t2 * t3;
    direction(1, 1) = t1 * t5 + t2 * t4 * t6;
    direction(1, 2) = t2 * t4 * t5 - t1 * t6;

    direction(2, 0) = -t4;
    direction(2, 1) = t3 * t6;
    direction(2, 2) = t3 * t5;

    new_image->SetDirection(image->GetDirection()*direction);

    
    return new_image;
}

constexpr size_t number_of_piramids = 4;

struct RegistrationParameters{
    size_t bin_numbers = 1;
    double relative_scales = 1;
    double learning_rate = 1;
    double sampling_percentage = .1;
    double relaxation_factor = 1;
    size_t convergence_window_size;
    size_t optimization_iterations = 1;
    std::array<size_t,number_of_piramids> piramid_sizes;
    std::array<size_t,number_of_piramids> bluering_sizes;
};

struct info_solve_registration
{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    const itk::Matrix<double> &initial_rotation;
    
};

std::tuple<double,TransformType::Pointer,TransformType::Pointer> solve_registration(const info_solve_registration &info_registration, const RegistrationParameters& parameters)
{
    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    metric->SetNumberOfHistogramBins(parameters.bin_numbers);

    metric->SetUseMovingImageGradientFilter(true);
    metric->SetUseFixedImageGradientFilter(true);
    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

     using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageType,
                                          ImageType>;

    auto initialTransform = TransformType::New();
    itk::Euler3DTransform<double>::Pointer matrix = itk::Euler3DTransform<double>::New();
    matrix->SetMatrix(info_registration.initial_rotation);
    //matrix->SetRotation(info_registration.initial_rotation[0] * (3.14159265359 / 180), info_registration.initial_rotation[1] * (3.14159265359 / 180), info_registration.initial_rotation[2] * (3.14159265359 / 180));
    TransformInitializerType::Pointer initializer =TransformInitializerType::New();
    initialTransform->SetMatrix(matrix->GetMatrix());
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(info_registration.fixed_image);
    initializer->SetMovingImage(info_registration.moving_image);
    initializer->InitializeTransform();
    initializer->GeometryOn();

    registration->SetFixedImage(info_registration.fixed_image);
    registration->SetMovingImage(info_registration.moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());

    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = parameters.relative_scales;
    optimizerScales[4] = parameters.relative_scales;
    optimizerScales[5] = parameters.relative_scales;

    optimizer->SetScales(optimizerScales);

    optimizer->SetNumberOfIterations(parameters.optimization_iterations);
    optimizer->SetLearningRate(parameters.learning_rate);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetReturnBestParametersAndValue(false);
    itk::SizeValueType value{parameters.convergence_window_size};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(parameters.relaxation_factor);

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(number_of_piramids);
    shrinkFactorsPerLevel[0] = parameters.piramid_sizes[0];
    shrinkFactorsPerLevel[1] = parameters.piramid_sizes[1];
    shrinkFactorsPerLevel[2] = parameters.piramid_sizes[2];
    shrinkFactorsPerLevel[3] = parameters.piramid_sizes[2];

    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(number_of_piramids);
    smoothingSigmasPerLevel[0] = parameters.bluering_sizes[0];
    smoothingSigmasPerLevel[1] = parameters.bluering_sizes[1];
    smoothingSigmasPerLevel[2] = parameters.bluering_sizes[2];
    smoothingSigmasPerLevel[3] = parameters.bluering_sizes[3];

    registration->SetNumberOfLevels(number_of_piramids);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::RANDOM;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(parameters.sampling_percentage);

    std::srand(std::time(nullptr)); 
    registration->MetricSamplingReinitializeSeed(std::rand());

    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        TransformType::Pointer finalTransform = registration->GetModifiableTransform();
        return {100.0, finalTransform,initialTransform};
    }

    TransformType::Pointer finalTransform = registration->GetModifiableTransform();
    return {optimizer->GetCurrentMetricValue(), finalTransform,initialTransform};
}

itk::PointSet<itk::Point<float, 3>, 3>::Pointer extract_point_cloud(ImageType::Pointer segmentedImage){
    using MeshType = itk::Mesh<float, 3>;
    using MeshSourceType = itk::BinaryMask3DMeshSource<ImageType, MeshType>;
    using PointType = itk::Point<float, 3>;
    using PointSetType = itk::PointSet<PointType, 3>;

    auto meshSource = MeshSourceType::New();
    meshSource->SetInput(segmentedImage);
    meshSource->Update();
    auto mesh = meshSource->GetOutput();

    auto pointCloud = PointSetType::New();
    auto points = mesh->GetPoints();

    PointSetType::PointsContainer::Pointer pointContainer = PointSetType::PointsContainer::New();
    pointContainer->Reserve(points->Size());

    auto pointIter = points->Begin();
    while (pointIter != points->End()) {
        PointType point = pointIter.Value();
        pointContainer->InsertElement(pointIter.Index(), point);
        ++pointIter;
    }
    pointCloud->SetPoints(pointContainer);
    //WritePointCloudToTxt(pointCloud, fileName);

    return pointCloud;
}

Eigen::Matrix3d NormalizeEigenvectors(const Eigen::Matrix3d& eigenvectors) {
    Eigen::Matrix3d normalized = eigenvectors;
    for (int i = 0; i < 3; ++i) {
        double maxAbsValue = 0.0;
        int maxIndex = 0;
        for (int j = 0; j < 3; ++j) {
            if (std::abs(normalized(j, i)) > maxAbsValue) {
                maxAbsValue = std::abs(normalized(j, i));
                maxIndex = j;
            }
        }
        if (normalized(maxIndex, i) < 0) {
            normalized.col(i) = -normalized.col(i);
        }
    }
    return normalized;
}


using PointSetType = itk::PointSet<itk::Point<float, 3>, 3>;

 std::pair<itk::Point<double, 3U>, Eigen::Matrix3d> PCA2 (PointSetType::Pointer pointCloud){
    using namespace Eigen;
	using namespace std;
    using PointSetType = itk::PointSet<float, 3>;
    using PointType = PointSetType::PointType;
    using PointsContainer = PointSetType::PointsContainer;
    using PointsIterator = PointsContainer::ConstIterator;

    vector<Vector3d> eigen_points;
    // Iterate through the points in the pointCloud
    PointsIterator it = pointCloud->GetPoints()->Begin();
    PointsIterator end = pointCloud->GetPoints()->End();

    for (; it != end; ++it){
        PointType itk_point = it.Value();
        Eigen::Vector3d eigen_point(itk_point[0], itk_point[1], itk_point[2]);
        eigen_points.push_back(eigen_point);
    };

    Vector3d mean = Vector3d::Zero();
    for (const auto& vec : eigen_points) {
        mean += vec;
    }
    mean /= eigen_points.size();

    itk::Point<double, 3> itk_mean;
    itk_mean[0] = mean[0];
    itk_mean[1] = mean[1];
    itk_mean[2] = mean[2];

    std::cout << "Centroid:" << std::endl;
    std::cout << mean << std::endl;

    //Centrar os dados tendo em conta a posição média
    MatrixXd centered_data(3, eigen_points.size());
    for (size_t i = 0; i < eigen_points.size(); ++i) {
        centered_data.col(i) = eigen_points[i] - mean;
    }

    //Covariance matrix
    MatrixXd covariance = (centered_data * centered_data.transpose()) / (eigen_points.size() - 1);
    //std::cout << "Covariance Matrix:\n" << covariance<< std::endl

    //Eigendecomposition da covariance matrix
    SelfAdjointEigenSolver<MatrixXd> eigensolver(covariance);

    //Eigenvectors e eigenvalues
    Vector3d eigenvalues = eigensolver.eigenvalues();
    Matrix3d eigenvectors = eigensolver.eigenvectors();

        // Normalize the eigenvectors
    eigenvectors = NormalizeEigenvectors(eigenvectors);

    //Pares de valores prórpios com o respetivo vetor próprio
    vector<pair<double, Vector3d>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.push_back(make_pair(eigenvalues(i), eigenvectors.col(i)));
    }
   
    //Lambda function para comparar os valores prórpios dos pares
    auto compare = [](const pair<double, Vector3d>& a, const pair<double, Vector3d>& b) {
        return a.first > b.first;
    };

    //Organiza os pares por ordem decrescente de valor próprio 
    sort(eigen_pairs.begin(), eigen_pairs.end(), compare);

    /* 
    //Printa os pares eigenvalue-eigenvector
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Sorted Eigenvalue-Eigenvector Pairs:" << endl;
    for (const auto& pair : eigen_pairs) {
        cout << "Eigenvalue: " << pair.first << ", Eigenvector: \n" << pair.second << endl;
    }
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Principal Component: " << eigen_pairs[0].second.transpose() << endl;
    */
    
    Matrix3d principal_components;
    for (int i = 0; i < 3; ++i) {
        principal_components.col(i) = eigen_pairs[i].second;
    }

    // Print the principal components matrix
    cout << "Principal Components Matrix:\n" << principal_components << endl;
    auto pair = make_pair(itk_mean, principal_components);

    return pair;
    }

itk::Matrix<double, 3, 3> CalculateRotationMatrix(Eigen::Matrix<double, 3, 3> movingPCA, Eigen::Matrix<double, 3, 3> fixedPCA) {
     auto rotation = fixedPCA * movingPCA.transpose();

    // Define the ITK matrix
    itk::Matrix<double, 3, 3> itk_matrix;

    // Convert Eigen matrix to ITK matrix by copying elements
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            itk_matrix[i][j] = rotation(i, j);
        }
    }
    return itk_matrix;
}

ImageType::Pointer pre_processing_us(ImageType::Pointer volume, int low_threshold, int upper_threshold, float sigma) {
    using LabelPixelType = unsigned int;
    using LabelImageType = itk::Image<LabelPixelType, 3>;
    using SmoothingFilterType = itk::SmoothingRecursiveGaussianImageFilter<ImageType, ImageType>;
    auto smoothingFilter = SmoothingFilterType::New();
    smoothingFilter->SetInput(volume);
    smoothingFilter->SetSigma(sigma);

    auto thresholdFilter = itk::BinaryThresholdImageFilter<ImageType, ImageType>::New();
    thresholdFilter->SetInput(smoothingFilter->GetOutput());
    thresholdFilter->SetLowerThreshold(low_threshold);
    thresholdFilter->SetUpperThreshold(upper_threshold);
    thresholdFilter->SetInsideValue(1);
    thresholdFilter->SetOutsideValue(0);

    using CastFilterType = itk::CastImageFilter<ImageType, LabelImageType>;
    auto castFilter = CastFilterType::New();
    castFilter->SetInput(thresholdFilter->GetOutput());
    using ConnectedComponentFilterType = itk::ConnectedComponentImageFilter<LabelImageType, LabelImageType>;
    auto connectedComponentFilter = ConnectedComponentFilterType::New();
    connectedComponentFilter->SetInput(castFilter->GetOutput());

    using RelabelFilterType = itk::RelabelComponentImageFilter<LabelImageType, LabelImageType>;
    auto relabelFilter = RelabelFilterType::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
    relabelFilter->SetMinimumObjectSize(500); 

    using ReverseCastFilterType = itk::CastImageFilter<LabelImageType, ImageType>;
    auto finalCastFilter = ReverseCastFilterType::New();
    finalCastFilter->SetInput(relabelFilter->GetOutput());

    auto finalThresholdFilter = itk::BinaryThresholdImageFilter<ImageType, ImageType>::New();
    finalThresholdFilter->SetInput(finalCastFilter->GetOutput());
    finalThresholdFilter->SetLowerThreshold(1);
    finalThresholdFilter->SetUpperThreshold(1);
    finalThresholdFilter->SetInsideValue(1);
    finalThresholdFilter->SetOutsideValue(0);

    using ContourFilterType = itk::BinaryContourImageFilter<ImageType, ImageType>;
    auto contourFilter = ContourFilterType::New();
    contourFilter->SetInput(finalThresholdFilter->GetOutput());
    contourFilter->SetForegroundValue(1);
    contourFilter->SetBackgroundValue(0);

    try {
        contourFilter->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return nullptr;
    }

    return contourFilter->GetOutput();
}

ImageType::Pointer pre_processing_ct(ImageType::Pointer volume, int low_threshold, int upper_threshold) {
    auto thresholdFilter = itk::BinaryThresholdImageFilter<ImageType, ImageType>::New();
    thresholdFilter->SetInput(volume);
    thresholdFilter->SetLowerThreshold(low_threshold);
    thresholdFilter->SetUpperThreshold(upper_threshold);
    thresholdFilter->SetInsideValue(1);
    thresholdFilter->SetOutsideValue(0);

    using ContourFilterType = itk::BinaryContourImageFilter<ImageType, ImageType>;
    auto contourFilter = ContourFilterType::New();
    contourFilter->SetInput(thresholdFilter->GetOutput());
    contourFilter->SetForegroundValue(1);
    contourFilter->SetBackgroundValue(0);

    try {
        contourFilter->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return nullptr;
    }

    return contourFilter->GetOutput();
}

int main(int argc, char **argv)
{

    
    if(argc!=4){
        if(argc>4 || argc == 1){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - input volume, (fixed)\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
            }
        if(argc == 2){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
        }
        if(argc == 3){
            std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - "<< std::string(argv[2]) << "\n"
                      << "third parameter - output volume";
                      return 1;
        }
    }
    
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();

    std::string dirName{argv[1]};
    fixedImageReader->SetFileName(dirName);

    std::string dirName2{argv[2]};
    movingImageReader->SetFileName(dirName2);

    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
    ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

    auto print_image_info = [](itk::Image<PixelType,3>::Pointer image, std::string name){
        std::cout << "-------------------\n";
        std::cout << "(" << name << ") :\n -------------------\n";
        std::cout << "direction:\n";
        auto direction = image->GetDirection();
        for(size_t i = 0; i < 3; ++i){
            for(size_t j = 0; j < 3; ++j)
                std::cout <<  direction(i,j) << " ";
            std::cout << "\n";
        }
        auto origin = image->GetOrigin();
        std::printf("\norigin: (%f %f %f)",image->GetOrigin()[0],image->GetOrigin()[1],image->GetOrigin()[2]);
        std::printf("\nspacing: (%f %f %f)",image->GetSpacing()[0],image->GetSpacing()[1],image->GetSpacing()[2]);
        std::printf("\nsize: (%d %d %d)",(int)image->GetLargestPossibleRegion().GetSize()[0],(int)image->GetLargestPossibleRegion().GetSize()[1],(int)image->GetLargestPossibleRegion().GetSize()[2]);
        std::cout << "\n-------------------\n";
    };

    print_image_info(pointer2fixedimage,"fixed image");
    print_image_info(pointer2movingimage,"moving image");

    std::array<Eigen::Vector3d,27> initial_configs;


    auto preprocessed_moving = pre_processing_us(pointer2movingimage, 80, 255, 2);
    if (!preprocessed_moving) {
        std::cerr << "Failed to preprocess moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto preprocessed_fixed = pre_processing_ct(pointer2fixedimage, -500, 168);
    if (!preprocessed_fixed) {
        std::cerr << "Failed to preprocess fixed volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto moving_pointcloud = extract_point_cloud(preprocessed_moving);
    if (!moving_pointcloud) {
    std::cerr << "Failed to extract moving pointcloud" << std::endl;
    return EXIT_FAILURE;
    }

    auto fixed_pointcloud = extract_point_cloud(preprocessed_fixed);
    if (!fixed_pointcloud) {
    std::cerr << "Failed to extract fixed pointcloud" << std::endl;
    return EXIT_FAILURE;

    } itk::Point<double, 3> movingCentroid;
    itk::Point<double, 3> fixedCentroid;

    auto movingPC_matrix = PCA2(moving_pointcloud);
    auto moving_principal_components = movingPC_matrix.second;
    auto moving_centroid = movingPC_matrix.first;
    
    auto fixedPC_matrix = PCA2(fixed_pointcloud);
    auto fixed_principal_components = fixedPC_matrix.second;
    auto fixed_centroid = fixedPC_matrix.first;

    auto rotationMatrix = CalculateRotationMatrix(moving_principal_components, fixed_principal_components);
    
    std::ofstream myfile;
    myfile.open("results_of_fullscale_optimization.csv");
    myfile << "run,bins,sampling percentage,relative_scales,learning rate,relaxation,convergence window,piramid sizes,bluring sizes,best cost,total time\n";

    constexpr size_t local_permut = 1;
/*
    std::array<size_t,local_permut> bin_numbers{10,20,50};
    std::array<double,local_permut> percentage_numbers{0.05,0.1,0.4};
    std::array<double,local_permut> relative_scales{2000.0,1000.0,800.0};
    std::array<double,local_permut> learning_rate{10,8,5};
    std::array<double,local_permut> relaxation_factor{0.9,0.7,0.5};
    std::array<size_t,local_permut> optimization_iterations{100,300,500};
    std::array<size_t,local_permut> convergence_window_size{5,10,20};
    std::array<std::array<size_t,4>,local_permut> piramid_sizes{{{4,3,1,0},{8,6,2,0},{8,4,2,0}}};
    std::array<std::array<size_t,4>,local_permut> bluering_sizes{{{6,3,2,0},{6,3,1,0},{8,4,2,0}}};
*/
    std::array<size_t,local_permut> bin_numbers{50};
    std::array<double,local_permut> percentage_numbers{0.02};
    std::array<double,local_permut> relative_scales{1000.0};
    std::array<double,local_permut> learning_rate{7};
    std::array<double,local_permut> relaxation_factor{0.7};
    std::array<size_t,local_permut> optimization_iterations{1000};
    std::array<size_t,local_permut> convergence_window_size{10};
    std::array<std::array<size_t,4>,local_permut> piramid_sizes{{{8,4,2,0}}};
    std::array<std::array<size_t,4>,local_permut> bluering_sizes{{{8,4,2,0}}};

    constexpr size_t total_permutations = bin_numbers.size()*percentage_numbers.size()*relative_scales.size()*learning_rate.size()*relaxation_factor.size()*convergence_window_size.size()*piramid_sizes.size()*bluering_sizes.size();

    auto run_parameterized_optimization = [&](size_t bins, size_t iters, double percentage, double relative_scales,double learning_rate, double relaxation_factor,size_t window_size, std::array<size_t,4> piramid_sizes, std::array<size_t,4> bluering_sizes){
        std::vector<std::tuple<double, TransformType::Pointer,TransformType::Pointer>> full_runs;
        {         
            std::mutex mut;
            auto pool = curan::utilities::ThreadPool::create(6,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
            //for (const auto &initial_config : initial_configs){
                curan::utilities::Job job{"solving registration",[&](){
                    auto solution = solve_registration({pointer2fixedimage, pointer2movingimage, rotationMatrix},{bins,relative_scales,learning_rate,percentage,relaxation_factor,window_size,iters,piramid_sizes,bluering_sizes});
                    {
                        std::lock_guard<std::mutex> g{mut};
                        full_runs.emplace_back(solution);
                        std::cout << "cost: <<" << std::get<0>(solution) << "\n";
                    }              
                }};
                pool->submit(job);
            //} 
        }
        return full_runs;
    };
    std::vector<std::tuple<double, TransformType::Pointer,TransformType::Pointer>> full_runs;
    size_t total_runs = 0;
    for(const auto& bin_n : bin_numbers)
        for(const auto& percent_n : percentage_numbers)
            for(const auto& rel_scale : relative_scales)
                for(const auto& learn_rate : learning_rate)
                    for(const auto& relax_factor : relaxation_factor)
                        for(const auto& wind_size : convergence_window_size)
                            for(const auto& pira_size : piramid_sizes)
                                for(const auto& iters : optimization_iterations)
                                    for(const auto& blur_size : bluering_sizes) {
                                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                        full_runs = run_parameterized_optimization(bin_n,iters,percent_n,rel_scale,learn_rate,relax_factor,wind_size,pira_size,blur_size);
                                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                        for(auto&& run : full_runs){
                                            myfile << total_runs << "," << bin_n << "," << percent_n << "," << rel_scale << "," << learn_rate << "," << relax_factor << "," << wind_size << ", {";
                                            for(const auto& val : pira_size)
                                                myfile << val << ";";
                                            myfile << "}, {";
                                            for(const auto& val : blur_size)
                                                myfile << val << ";"; 
                                            myfile <<"}," << std::get<0>(run) << "," << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
                                            std::printf("\n==(%d/%d)==\n",(int)total_runs,(int)total_permutations);
                                        }
                                        ++total_runs;
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


    std::printf("Choosen cost: %.2f\n",minimum_val);
    
    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    std::cout << "best estimated transform out of " << initial_configs.size() << " is: \n" << std::get<1>(full_runs[minimum_index]) << " and other is: " << std::get<2>(full_runs[minimum_index]) ;

    using CompositeTransformType = itk::CompositeTransform<double, Dimension>;
    CompositeTransformType::Pointer outputCompositeTransform =
    CompositeTransformType::New();
    //outputCompositeTransform->AddTransform(std::get<2>(full_runs[minimum_index]));
    //outputCompositeTransform->AddTransform());

    using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
 
    {
        auto resample = ResampleFilterType::New();
        resample->SetTransform(std::get<1>(full_runs[minimum_index]));
        resample->SetInput(pointer2movingimage);
 
        resample->SetSize(pointer2fixedimage->GetLargestPossibleRegion().GetSize());
        resample->SetOutputOrigin(pointer2fixedimage->GetOrigin());
        resample->SetOutputSpacing(pointer2fixedimage->GetSpacing());
        resample->SetOutputDirection(pointer2fixedimage->GetDirection());
        resample->SetDefaultPixelValue(0);

 
        using WriterType = itk::ImageFileWriter<ImageType>;
 
        auto writer = WriterType::New();
 
        writer->SetFileName(argv[3]);
        //writer->SetFileName("output_test_1.mha");

        writer->SetInput(resample->GetOutput());

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
    }

    {
        auto writer = WriterType::New();
 
        writer->SetFileName("moving"+std::string{argv[3]});
        writer->SetInput(pointer2movingimage);

        try{
            writer->Update();
        }
        catch (...){
            std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n";
            return 1;
        }    
    }

    return 0;
}