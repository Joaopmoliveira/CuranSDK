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


using PixelType = float;
using ImageType = itk::Image<PixelType, 3>;

ImageType::Pointer read_volume(const std::string& image_path) {
    using ImageReaderType = itk::ImageFileReader<ImageType>;

    auto imageReader = ImageReaderType::New();
    imageReader->SetFileName(image_path);

    try {
        imageReader->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return nullptr;
    }

    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(imageReader->GetOutput());
    minMaxCalculator->Compute();

    PixelType minValue = minMaxCalculator->GetMinimum();
    PixelType maxValue = minMaxCalculator->GetMaximum();

    std::cout << "Minimum pixel value: " << minValue << std::endl;
    std::cout << "Maximum pixel value: " << maxValue << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    return imageReader->GetOutput();
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

int write_volume(ImageType::Pointer volume, const std::string& path) {
    auto writer = itk::ImageFileWriter<ImageType>::New();
    writer->SetFileName(path);
    writer->SetInput(volume);

    try {
        writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Volume written in " << path << std::endl;
    return EXIT_SUCCESS;
}

using PointSetType = itk::PointSet<itk::Point<float, 3>, 3>;
void WritePointCloudToTxt(PointSetType::Pointer pointCloud, const std::string& fileName) {
    std::ofstream outFile(fileName);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << fileName << " for writing." << std::endl;
        return;
    }

    PointSetType::PointsContainer::ConstPointer points = pointCloud->GetPoints();
    for (auto it = points->Begin(); it != points->End(); ++it) {
        PointSetType::PointType point = it->Value();
        outFile << point[0] << " " << point[1] << " " << point[2] << std::endl;
    }

    outFile.close();
    std::cout << "Point cloud written to " << fileName << std::endl;
}

itk::PointSet<itk::Point<float, 3>, 3>::Pointer extract_point_cloud(ImageType::Pointer segmentedImage, std::string& fileName){
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
    WritePointCloudToTxt(pointCloud, fileName);

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
#include "itkEuler3DTransform.h"

void RotationMatrixToEulerAngles(const itk::Matrix<double, 3, 3>& R, double& alpha, double& beta, double& gamma)
{
    beta = std::atan2(-R[2][0], std::sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));

    if (std::cos(beta) != 0) {
        alpha = std::atan2(R[2][1], R[2][2]);
        gamma = std::atan2(R[1][0], R[0][0]);
    } else {
        alpha = 0;
        gamma = std::atan2(-R[0][1], R[1][1]);
    }
}

ImageType::Pointer transform_and_resample(
    ImageType::Pointer inputImage,
    ImageType::Pointer fixedImage,
    const itk::Point<double, 3>& fixedCentroid,
    const itk::Point<double, 3>& movingCentroid,
    const itk::Matrix<double, 3, 3>& rotationMatrix) {
/*
    // Define the transform
    using TransformType = itk::Euler3DTransform<double>;
    TransformType::Pointer transform = TransformType::New();
    transform->SetCenter(movingCentroid);

    double alpha, beta, gamma;
    RotationMatrixToEulerAngles(rotationMatrix, alpha, beta, gamma);
    // Set rotation angles
    const double radiansX = 20 * itk::Math::pi / 180.0;
    const double radiansY = 0 * itk::Math::pi / 180.0;
    const double radiansZ = 0 * itk::Math::pi / 180.0;
    transform->SetRotation(alpha, beta, gamma);

    // Set translation
    TransformType::OutputVectorType translation;

    auto T = fixedCentroid - rotationMatrix * movingCentroid;
    transform->SetTranslation(T);
    */

       // Define the transform
    using TransformType = itk::VersorRigid3DTransform<double>;
    TransformType::Pointer transform = TransformType::New();
    transform->SetCenter(movingCentroid);

    itk::Versor<double> versor;
    versor.Set(rotationMatrix);
    transform->SetRotation(versor);
    TransformType::OutputVectorType translation = fixedCentroid - rotationMatrix * movingCentroid;
    transform->SetTranslation(translation);

    using ResampleFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    auto resampleFilter = ResampleFilterType::New();
    resampleFilter->SetInput(inputImage);
    resampleFilter->SetTransform(transform);

    resampleFilter->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
    resampleFilter->SetOutputSpacing(fixedImage->GetSpacing());
    resampleFilter->SetOutputOrigin(fixedImage->GetOrigin());
    resampleFilter->SetOutputDirection(fixedImage->GetDirection());

    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
    auto interpolator = InterpolatorType::New();
    resampleFilter->SetInterpolator(interpolator);

    try {
        resampleFilter->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error during resampling: " << error << std::endl;
        return nullptr;
    }

    return resampleFilter->GetOutput();
}

int main(){
    std::string moving = "/precious_phantom/output_volume_0,0,350_454,244,567.mha";
    std::string fixed = "/precious_phantom/output_volume_0,0,90_512,200,197.mha";
    std::string moving_preprocessed = "/precious_phantom/moving_preprocessed.mha";
    std::string fixed_preprocessed = "/precious_phantom/fixed_preprocessed.mha";
    std::string moving_pointcloudd = "/precious_phantom/moving_pointcloud.txt";
    std::string fixed_pointcloudd = "/precious_phantom/fixed_pointcloud.txt";
    std::string output_transformed_imagee = "/precious_phantom/moving_transformed.mha";
    std::string moving_image_path = CURAN_COPIED_RESOURCE_PATH + moving;
    std::string fixed_image_path = CURAN_COPIED_RESOURCE_PATH + fixed;
    std::string moving_preprocessed_path = CURAN_COPIED_RESOURCE_PATH + moving_preprocessed;
    std::string fixed_preprocessed_path = CURAN_COPIED_RESOURCE_PATH + fixed_preprocessed;
    std::string moving_pointcloud_path = CURAN_COPIED_RESOURCE_PATH + moving_pointcloudd;
    std::string fixed_pointcloud_path = CURAN_COPIED_RESOURCE_PATH + fixed_pointcloudd;
    std::string output_transformed_image_path = CURAN_COPIED_RESOURCE_PATH + output_transformed_imagee;


    auto moving_volume = read_volume(moving_image_path);
    if (!moving_volume) {
        std::cerr << "Failed to read moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto fixed_volume = read_volume(fixed_image_path);
    if (!fixed_volume) {
        std::cerr << "Failed to read fixed volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto preprocessed_moving = pre_processing_us(moving_volume, 80, 255, 2);
    if (!preprocessed_moving) {
        std::cerr << "Failed to preprocess moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto preprocessed_fixed = pre_processing_ct(fixed_volume, -500, 168);
    if (!preprocessed_fixed) {
        std::cerr << "Failed to preprocess fixed volume" << std::endl;
        return EXIT_FAILURE;
    }

    if (write_volume(preprocessed_moving, moving_preprocessed_path) != EXIT_SUCCESS) {
        std::cerr << "Failed to write preprocessed moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    if (write_volume(preprocessed_fixed, fixed_preprocessed_path) != EXIT_SUCCESS) {
        std::cerr << "Failed to write preprocessed fixed volume" << std::endl;
        return EXIT_FAILURE;
    }

    auto moving_pointcloud = extract_point_cloud(preprocessed_moving, moving_pointcloud_path);
    if (!moving_pointcloud) {
    std::cerr << "Failed to extract moving pointcloud" << std::endl;
    return EXIT_FAILURE;
    }

    auto fixed_pointcloud = extract_point_cloud(preprocessed_fixed, fixed_pointcloud_path);
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


    std::cout << "Rotation Matrix:" << std::endl;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            std::cout << rotationMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;


    
    auto transformed_moving_volume = transform_and_resample(moving_volume, fixed_volume, fixed_centroid, moving_centroid, rotationMatrix);
    if (!transformed_moving_volume) {
        std::cerr << "Failed to transform and resample moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    if (write_volume(transformed_moving_volume, output_transformed_image_path) != EXIT_SUCCESS) {
        std::cerr << "Failed to write transformed moving volume" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Transformed moving volume written to " << output_transformed_image_path << std::endl;



    return 0;
}

