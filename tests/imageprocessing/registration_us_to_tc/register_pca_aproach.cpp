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

itk::Matrix<double, 3, 3> PCA(itk::PointSet<itk::Point<float, 3>, 3>::Pointer pointCloud) {
    using PointType = itk::Point<float, 3>;
    unsigned int numberOfPoints = pointCloud->GetNumberOfPoints();

    // centroid
    PointType centroid;
    centroid.Fill(0.0);
    for (unsigned int i = 0; i < numberOfPoints; ++i) {
        PointType point = pointCloud->GetPoint(i);
        for (unsigned int j = 0; j < 3; ++j) {
            centroid[j] += point[j];
        }
    }
    for (unsigned int j = 0; j < 3; ++j) {
        centroid[j] /= numberOfPoints;
    }

    //  covariance matrix
    itk::Matrix<double, 3, 3> covarianceMatrix;
    covarianceMatrix.Fill(0.0);
    for (unsigned int i = 0; i < numberOfPoints; ++i) {
        PointType point = pointCloud->GetPoint(i);
        for (unsigned int j = 0; j < 3; ++j) {
            for (unsigned int k = 0; k < 3; ++k) {
                covarianceMatrix[j][k] += (point[j] - centroid[j]) * (point[k] - centroid[k]);
            }
        }
    }
    for (unsigned int j = 0; j < 3; ++j) {
        for (unsigned int k = 0; k < 3; ++k) {
            covarianceMatrix[j][k] /= (numberOfPoints - 1);
        }
    }

    vnl_matrix<double> vnlCovarianceMatrix(3, 3);
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            vnlCovarianceMatrix[i][j] = covarianceMatrix[i][j];
        }
    }    vnl_symmetric_eigensystem<double> eigensystem(vnlCovarianceMatrix);

    itk::Matrix<double, 3, 3> principalAxes;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            principalAxes[j][i] = eigensystem.get_eigenvector(i)[j];
        }
    }

    std::cout << "Principal Axes:" << std::endl;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            std::cout << principalAxes[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return principalAxes;
}

itk::Matrix<double, 3, 3> GetRotationMatrix(itk::Matrix<double, 3, 3> &source, itk::Matrix<double, 3, 3> &target) {
    itk::Matrix<double, 3, 3> rotationMatrix;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            rotationMatrix[i][j] = 0.0;
            for (unsigned int k = 0; k < 3; ++k) {
                rotationMatrix[i][j] += target[i][k] * source[k][j];
            }
        }
    }

    std::cout << "Rotation Matrix:" << std::endl;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            std::cout << rotationMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return rotationMatrix;
}



class CommandIterationUpdate : public itk::Command
{
public:
  using Self = CommandIterationUpdate;
       using Superclass = itk::Command;
  using Pointer = itk::SmartPointer<Self>;
  itkNewMacro(Self);
 
protected:
  CommandIterationUpdate() = default;
 
public:
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using OptimizerPointer = const OptimizerType *;
  void
  Execute(itk::Object * caller, const itk::EventObject & event) override
  {
    Execute((const itk::Object *)caller, event);
  }
  void
  Execute(const itk::Object * object, const itk::EventObject & event) override
  {
    auto optimizer = static_cast<OptimizerPointer>(object);
    if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    }
    std::cout << optimizer->GetCurrentIteration() << "   ";
    std::cout << optimizer->GetValue() << "   ";
    std::cout << optimizer->GetCurrentPosition() << std::endl;
  }
};


int RegisterImages(const std::string& fixedImagePath, const std::string& movingImagePath, itk::Matrix<double>& rotationMatrix){

  constexpr unsigned int Dimension = 3;
  using PixelType = float;
  using FixedImageType = itk::Image<PixelType, Dimension>;
  using MovingImageType = itk::Image<PixelType, Dimension>;
  using TransformType = itk::VersorRigid3DTransform<double>;
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using MetricType = itk::MattesMutualInformationImageToImageMetricv4<FixedImageType, MovingImageType>;
  using InterpolatorType = itk::LinearInterpolateImageFunction<FixedImageType, double>;
  using RegistrationType = itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType, TransformType>;
using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
  using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
    using TransformInitializerType = itk::CenteredTransformInitializer<TransformType,FixedImageType,MovingImageType>;
  using VersorType = TransformType::VersorType;
  using VectorType = VersorType::VectorType;
  using OptimizerScalesType = OptimizerType::ScalesType;


  auto metric = MetricType::New();
  auto optimizer = OptimizerType::New();
  auto registration = RegistrationType::New();

    auto fixedInterpolator = InterpolatorType::New();
    auto movingInterpolator = InterpolatorType::New();

    unsigned int numberOfBins = 50;
    metric->SetNumberOfHistogramBins(numberOfBins);
    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);
    metric->SetFixedInterpolator(fixedInterpolator);
    metric->SetMovingInterpolator(movingInterpolator);
    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
    RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    double samplingPercentage = 0.30;


 
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
      registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);
    registration->MetricSamplingReinitializeSeed(121213);

  auto initialTransform = TransformType::New();
  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();
 
  fixedImageReader->SetFileName(fixedImagePath);
  movingImageReader->SetFileName(movingImagePath);
 
  registration->SetFixedImage(fixedImageReader->GetOutput());
  registration->SetMovingImage(movingImageReader->GetOutput());


  auto initializer = TransformInitializerType::New();
  initializer->SetTransform(initialTransform);
  initializer->SetFixedImage(fixedImageReader->GetOutput());
  initializer->SetMovingImage(movingImageReader->GetOutput());
  initializer->MomentsOn();
  initializer->InitializeTransform();



  VersorType rotation;
  VectorType axis;
  axis[0] = 0.0;
  axis[1] = 0.0;
  axis[2] = 1.0;
  constexpr double angle = 0;
  rotation.Set(axis, angle);

  initialTransform->SetRotation(rotation);
//initialTransform->SetMatrix(rotationMatrix);


  registration->SetInitialTransform(initialTransform);

 

  OptimizerScalesType optimizerScales(
    initialTransform->GetNumberOfParameters());
  const double translationScale = 1.0 / 1000.0;
  optimizerScales[0] = 10.0;
  optimizerScales[1] = 10.0;
  optimizerScales[2] = 10.0;
  optimizerScales[3] = translationScale;
  optimizerScales[4] = translationScale;
  optimizerScales[5] = translationScale;
  optimizer->SetScales(optimizerScales);
  optimizer->SetNumberOfIterations(5);
  optimizer->SetLearningRate(40);
  optimizer->SetMinimumStepLength(0.001);
  optimizer->SetReturnBestParametersAndValue(true);
 

  auto observer = CommandIterationUpdate::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);
 
  constexpr unsigned int numberOfLevels = 1;
 
  RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
  shrinkFactorsPerLevel.SetSize(1);
  shrinkFactorsPerLevel[0] = 1;
 
  RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
  smoothingSigmasPerLevel.SetSize(1);
  smoothingSigmasPerLevel[0] = 0;
 
  registration->SetNumberOfLevels(numberOfLevels);
  registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
  registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);
 
  try
  {
    registration->Update();
    std::cout << "Optimizer stop condition: "
              << registration->GetOptimizer()->GetStopConditionDescription()
              << std::endl;
  }
  catch (const itk::ExceptionObject & err)
  {
    std::cerr << "ExceptionObject caught !" << std::endl;
    std::cerr << err << std::endl;
    return EXIT_FAILURE;
  }
 
  const TransformType::ParametersType finalParameters =
    registration->GetOutput()->Get()->GetParameters();
 
  const double       versorX = finalParameters[0];
  const double       versorY = finalParameters[1];
  const double       versorZ = finalParameters[2];
  const double       finalTranslationX = finalParameters[3];
  const double       finalTranslationY = finalParameters[4];
  const double       finalTranslationZ = finalParameters[5];
  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();
  const double       bestValue = optimizer->GetValue();
 

  std::cout << std::endl << std::endl;
  std::cout << "Result = " << std::endl;
  std::cout << " versor X      = " << versorX << std::endl;
  std::cout << " versor Y      = " << versorY << std::endl;
  std::cout << " versor Z      = " << versorZ << std::endl;
  std::cout << " Translation X = " << finalTranslationX << std::endl;
  std::cout << " Translation Y = " << finalTranslationY << std::endl;
  std::cout << " Translation Z = " << finalTranslationZ << std::endl;
  std::cout << " Iterations    = " << numberOfIterations << std::endl;
  std::cout << " Metric value  = " << bestValue << std::endl;
 

  auto finalTransform = TransformType::New();
 
  finalTransform->SetFixedParameters(
    registration->GetOutput()->Get()->GetFixedParameters());
  finalTransform->SetParameters(finalParameters);
 
  // Software Guide : BeginCodeSnippet
  TransformType::MatrixType matrix = finalTransform->GetMatrix();
  TransformType::OffsetType offset = finalTransform->GetOffset();
  std::cout << "Matrix = " << std::endl << matrix << std::endl;
  std::cout << "Offset = " << std::endl << offset << std::endl;

  using ResampleFilterType =
    itk::ResampleImageFilter<MovingImageType, FixedImageType>;
 
  auto resampler = ResampleFilterType::New();
 
  resampler->SetTransform(finalTransform);
  resampler->SetInput(movingImageReader->GetOutput());
 
  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();
 
  resampler->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
  resampler->SetOutputOrigin(fixedImage->GetOrigin());
  resampler->SetOutputSpacing(fixedImage->GetSpacing());
  resampler->SetOutputDirection(fixedImage->GetDirection());
  resampler->SetDefaultPixelValue(100);
 
 // Write result
    auto writer = itk::ImageFileWriter<MovingImageType>::New();
    std::string aux = "/precious_phantom/outputRegisteredImage.mha";
    std::string output_image_path = CURAN_COPIED_RESOURCE_PATH + aux;
    writer->SetFileName(output_image_path);
    writer->SetInput(resampler->GetOutput());

    try {
        writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }
  return 0;
}

int main(){
    std::string moving = "/precious_phantom/output_volume_0,100,350_454,244,567.mha";
    std::string fixed = "/precious_phantom/output_volume_0,0,350_454,244,567.mha";
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
    }

    auto pca_moving = PCA(moving_pointcloud);
    auto pca_fixed = PCA(fixed_pointcloud);
    auto rotationMatrix = GetRotationMatrix(pca_moving, pca_fixed);

    //RegisterImages(fixed_image_path, moving_image_path, rotationMatrix);

    return 0;
}

