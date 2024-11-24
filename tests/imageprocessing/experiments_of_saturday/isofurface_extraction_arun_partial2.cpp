#include <iostream>
#include <string>

#include "imageprocessing/VolumetricRegistration.h"
#include "itkAutomaticTopologyMeshSource.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkBinaryDilateImageFilter.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinomialBlurImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCenteredTransformInitializer.h"
#include "itkCommand.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkCorrelationImageToImageMetricv4.h"
#include "itkDemonsImageToImageMetricv4.h"
#include "itkEuclideanDistancePointMetric.h"
#include "itkEuler3DTransform.h"
#include "itkExtractImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkHistogram.h"
#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkImageMaskSpatialObject.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkImageSeriesReader.h"
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkImageToListSampleAdaptor.h"
#include "itkJointHistogramMutualInformationImageToImageMetricv4.h"
#include "itkLaplacianImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkMaskImageFilter.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkMeshSpatialObject.h"
#include "itkNormalQuadEdgeMeshFilter.h"
#include "itkNormalVariateGenerator.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkPointSetToPointSetRegistrationMethod.h"
#include "itkQuadEdgeMesh.h"
#include "itkQuadEdgeMeshExtendedTraits.h"
#include "itkLabelStatisticsImageFilter.h"
#include "itkRegionOfInterestImageFilter.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkRelabelComponentImageFilter.h"
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkSampleToHistogramFilter.h"
#include "itkScalarImageToHistogramGenerator.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkSpatialObjectReader.h"
#include "itkStatisticsImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkThresholdImageFilter.h"
#include "itkVector.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorIndexSelectionCastImageFilter.h"
#include "itkVersorRigid3DTransform.h"
#include "itkZeroCrossingBasedEdgeDetectionImageFilter.h"
#include "utils/TheadPool.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <itkANTSNeighborhoodCorrelationImageToImageMetricv4.h>
#include <itkTransformGeometryImageFilter.h>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <type_traits>

#include "imageprocessing/ArunAlgorithm.h"

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
  poorly_constrained_direction << 1.0, 0.0, 0.0, 0.0, std::cos(alpha),
      -std::sin(alpha), 0.0, std::sin(alpha), std::cos(alpha);
  Eigen::Matrix<double, 4, 4> T_rotation_extra =
      Eigen::Matrix<double, 4, 4>::Identity();
  T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
  return T_rotation_extra;
};

auto transform_y = [](double alpha)
{
  Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
  poorly_constrained_direction << std::cos(alpha), 0.0, std::sin(alpha), 0.0,
      1.0, 0.0, -std::sin(alpha), 0.0, std::cos(alpha);
  Eigen::Matrix<double, 4, 4> T_rotation_extra =
      Eigen::Matrix<double, 4, 4>::Identity();
  T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
  return T_rotation_extra;
};

auto transform_z = [](double alpha)
{
  Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
  poorly_constrained_direction << std::cos(alpha), -std::sin(alpha), 0.0,
      std::sin(alpha), std::cos(alpha), 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 4, 4> T_rotation_extra =
      Eigen::Matrix<double, 4, 4>::Identity();
  T_rotation_extra.block<3, 3>(0, 0) = poorly_constrained_direction;
  return T_rotation_extra;
};

Eigen::Matrix<double, 1, 9>
get_error(Eigen::Matrix<double, 4, 4> moving_to_fixed)
{
  Eigen::Matrix<double, 3, 9> ct_points;
  ct_points << 51.0910, 45.7005, 18.7328, -26.3736, -53.1790, -53.1238,
      -38.7812, 5.9589, -26.4164, 80.3660, 128.6975, 156.8359, 158.2443,
      127.2275, 87.3330, 67.2878, 99.4279, 129.4901, 517.6372, 516.1141,
      514.3752, 514.4684, 515.4078, 517.3948, 519.6219, 505.9474, 509.5589;

  Eigen::Matrix<double, 3, 9> world_points;
  world_points << -674.874, -714.276, -741.764, -741.121, -716.382, -677.195,
      -660.926, -683.793, -713.982, 90.9651, 88.4241, 58.843, 15.514, -11.3519,
      -9.55646, 3.30793, 50.3275, 16.0374, 31.3693, 8.2447, -2.56189, -5.17164,
      11.5598, 26.49, 36.5624, 10.8589, 2.87831;

  Eigen::Matrix<double, 3, 9> moved =
      (moving_to_fixed.block<3, 3>(0, 0) * ct_points).colwise() +
      moving_to_fixed.block<3, 1>(0, 3);

  Eigen::Matrix<double, 3, 9> error = world_points - moved;
  Eigen::Array<double, 1, 9> rooted =
      error.array().square().colwise().sum().sqrt();
  return rooted.matrix();
}

// Lamda to get a rotation matrix from a given angle but with a flipped
// principal direction
auto transform_flipped_principal_component = [](double alpha)
{
  Eigen::Matrix<double, 3, 3> poorly_constrained_direction;
  poorly_constrained_direction << 1.0, 0.0, 0.0, 0.0, std::cos(alpha),
      -std::sin(alpha), 0.0, std::sin(alpha), std::cos(alpha);

  Eigen::Matrix<double, 3, 3> flipping_direction;
  flipping_direction << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix<double, 4, 4> T_rotation_extra =
      Eigen::Matrix<double, 4, 4>::Identity();
  T_rotation_extra.block<3, 3>(0, 0) =
      poorly_constrained_direction * flipping_direction;
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

  ExtractionSurfaceInfo(size_t in_connected_components, double in_frequency,
                        std::string in_appendix, double in_reduction_factor,
                        int in_buffer_of_mask)
      : connected_components{in_connected_components}, frequency{in_frequency},
        appendix{in_appendix}, reduction_factor{in_reduction_factor},
        buffer_of_mask{in_buffer_of_mask} {};
};



void write_point_cloud(Eigen::Matrix<double, 3, Eigen::Dynamic> &rotated_point_cloud, const std::string &path)
{
  std::ofstream out{path};
  if (!out.is_open())
    throw std::runtime_error("failure to open output file");

  out << rotated_point_cloud;
};

itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>::Pointer
convert_mesh(const itk::Mesh<double>::Pointer &mesh)
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
    void Visit(unsigned long cellId, TriangleType *t)
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

    void set_required_data(
        itk::Mesh<double>::Pointer ref_inmesh,
        itk::QuadEdgeMesh<double, 3>::Pointer ref_outmesh,
        itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer in_container)
    {
      inmesh = ref_inmesh;
      outmesh = ref_outmesh;
      outmesh->SetPoints(in_container);
    }

    using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
    void Visit(unsigned long cellId, TriangleType *t)
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

  itk::QuadEdgeMesh<double, 3>::Pointer out_mesh =
      itk::QuadEdgeMesh<double, 3>::New();
  itk::QuadEdgeMesh<double, 3>::PointsContainer::Pointer container;
  {
    using TriangleType = itk::TriangleCell<itk::Mesh<double>::CellType>;
    using TriangleVisitorInterfaceType =
        itk::CellInterfaceVisitorImplementation<
            itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits,
            TriangleType, MeshTriangleVisitor>;
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
    using TriangleVisitorInterfaceType =
        itk::CellInterfaceVisitorImplementation<
            itk::Mesh<double>::PixelType, itk::Mesh<double>::CellTraits,
            TriangleType, MeshTriangleConstructorVisitor>;
    auto triangleVisitor = TriangleVisitorInterfaceType::New();
    triangleVisitor->set_required_data(mesh, out_mesh, container);
    using CellMultiVisitorType = itk::Mesh<double>::CellType::MultiVisitor;
    auto multiVisitor = CellMultiVisitorType::New();
    multiVisitor->AddVisitor(triangleVisitor);
    mesh->Accept(multiVisitor);
  }
  auto computation_of_normals = itk::NormalQuadEdgeMeshFilter<
      itk::QuadEdgeMesh<double, 3>,
      itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::New();
  itk::NormalQuadEdgeMeshFilter<
      itk::QuadEdgeMesh<double, 3>,
      itk::QuadEdgeMesh<itk::Vector<double, 3>, 3>>::WeightEnum weight_type =
      itk::NormalQuadEdgeMeshFilterEnums::Weight::THURMER;
  computation_of_normals->SetInput(out_mesh);
  computation_of_normals->SetWeight(weight_type);
  update_ikt_filter(computation_of_normals);
  return computation_of_normals->GetOutput();
}

Eigen::Matrix<double,3,Eigen::Dynamic> prune_surface(itk::Mesh<double>::Pointer mesh)
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
  point_set->SetPoints(point_container);

  index = 0;
  Eigen::Matrix<double,3,Eigen::Dynamic> points = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,index);
  for(const auto& point : (*point_container)){
    points.col(index) = Eigen::Matrix<double,3,1>{{point[0],point[1],point[2]}};
    ++index;
  }
  
  return points;
}

template <typename pixel_type>
int modify_image_with_transform(
    Eigen::Matrix<double, 4, 4> transform,
    typename itk::Image<pixel_type, 3>::Pointer image)
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
void print_image_with_transform(
    typename itk::Image<pixel_type, 3>::Pointer image,
    const std::string &image_path)
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

struct RegistrationData
{
  // internal data pre-allocated to save time
  using MaskType = itk::ImageMaskSpatialObject<3>;
  using ImageType = typename info_solve_registration<PixelType>::ImageType;
  using ImageRegistrationType =
      typename info_solve_registration<PixelType>::ImageType;
  using TransformType = itk::VersorRigid3DTransform<double>;
  using InterpolatorType =
      itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using MetricType =
      itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                       ImageRegistrationType>;
  using RegistrationType =
      itk::ImageRegistrationMethodv4<ImageRegistrationType,
                                     ImageRegistrationType, TransformType>;

  RegistrationData() {};
};

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

  RegistrationParameters(size_t in_bin_numbers, double in_relative_scales,
                         double in_learning_rate, double in_sampling_percentage,
                         double in_relaxation_factor,
                         size_t in_convergence_window_size,
                         size_t in_optimization_iterations,
                         std::array<size_t, size_info> in_piramid_sizes,
                         std::array<double, size_info> in_bluering_sizes)
      : bin_numbers{in_bin_numbers}, relative_scales{in_relative_scales},
        learning_rate{in_learning_rate},
        sampling_percentage{in_sampling_percentage},
        relaxation_factor{in_relaxation_factor},
        convergence_window_size{in_convergence_window_size},
        optimization_iterations{in_optimization_iterations},
        piramid_sizes{in_piramid_sizes}, bluering_sizes{in_bluering_sizes} {}
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
  void Execute(itk::Object *object, const itk::EventObject &event) override
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
      optimizer->SetMinimumStepLength(optimizer->GetMinimumStepLength() * 0.2);
      optimizer->SetMaximumStepSizeInPhysicalUnits(
          optimizer->GetMaximumStepSizeInPhysicalUnits() * 0.2);
    }
  }

  void Execute(const itk::Object *, const itk::EventObject &) override
  {
    return;
  }
};

template <typename PixelType>
std::tuple<double, Eigen::Matrix<double, 4, 4>, Eigen::Matrix<double, 4, 4>>
solve_registration(const info_solve_registration<PixelType> &info_registration,
                   const RegistrationParameters &parameters)
{
  using MaskType = itk::ImageMaskSpatialObject<3>;
  using ImageType = typename info_solve_registration<PixelType>::ImageType;
  using ImageRegistrationType =
      typename info_solve_registration<PixelType>::ImageType;
  using TransformType = itk::VersorRigid3DTransform<double>;
  using InterpolatorType =
      itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
  using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
  using MetricType =
      itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                       ImageRegistrationType>;
  using RegistrationType =
      itk::ImageRegistrationMethodv4<ImageRegistrationType,
                                     ImageRegistrationType, TransformType>;

  auto metric = MetricType::New();
  auto optimizer = OptimizerType::New();
  auto registration = RegistrationType::New();
  typename InterpolatorType::Pointer interpolator_moving =
      InterpolatorType::New();
  typename InterpolatorType::Pointer interpolator_fixed =
      InterpolatorType::New();

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
  itk::Euler3DTransform<double>::Pointer matrix =
      itk::Euler3DTransform<double>::New();
  itk::Vector<double, 3> origin;
  itk::Matrix<double> direction;

  for (size_t row = 0; row < 3; ++row)
  {
    origin[row] = info_registration.initial_rotation(row, 3);
    for (size_t col = 0; col < 3; ++col)
      direction(row, col) = info_registration.initial_rotation(row, col);
  }

  Eigen::Matrix<double, 4, 4> transformation =
      info_registration.initial_rotation;

  if (!(transformation.block<3, 3>(0, 0).transpose() *
        transformation.block<3, 3>(0, 0))
           .isDiagonal())
  {
    std::cout << "failure to initialize rotation matrix...\n";
    std::cout << "values are: \n";
    std::cout << transformation.block<3, 3>(0, 0) << std::endl;
    std::cout << "the multiplication with itself is:";
    std::cout << transformation.block<3, 3>(0, 0).transpose() *
                     transformation.block<3, 3>(0, 0)
              << std::endl;
    throw std::runtime_error("failure to initialize the rotation matrix");
  }
  initialTransform->SetMatrix(direction);
  initialTransform->SetTranslation(origin);

  registration->SetFixedImage(info_registration.fixed_image);
  registration->SetMovingImage(info_registration.moving_image);
  registration->SetInitialTransform(initialTransform);

  using OptimizerScalesType = OptimizerType::ScalesType;
  OptimizerScalesType optimizerScales(
      initialTransform->GetNumberOfParameters());

  optimizerScales[0] = 1.0;
  optimizerScales[1] = 1.0;
  optimizerScales[2] = 1.0;
  optimizerScales[3] = 1.0 / parameters.relative_scales;
  optimizerScales[4] = 1.0 / parameters.relative_scales;
  optimizerScales[5] = 1.0 / parameters.relative_scales;

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
    typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::NONE;
    registration->SetMetricSamplingStrategy(samplingStrategy);
  }
  else
  {
    typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::REGULAR;
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
  catch (...)
  {
    std::cout << "exception thrown" << std::endl;
    return {100.0, Eigen::Matrix<double, 4, 4>::Identity(),
            info_registration.initial_rotation};
  }
  TransformType::Pointer final_registration =
      registration->GetModifiableTransform();
  Eigen::Matrix<double, 4, 4> final_transformation =
      Eigen::Matrix<double, 4, 4>::Identity();
  for (size_t row = 0; row < 3; ++row)
  {
    final_transformation(row, 3) = final_registration->GetOffset()[row];
    for (size_t col = 0; col < 3; ++col)
      final_transformation(row, col) =
          final_registration->GetMatrix()(row, col);
  }
  return {optimizer->GetValue(), final_transformation,
          info_registration.initial_rotation};
}

void write_point_set(const std::string &filename,
                     itk::PointSet<double, 3>::Pointer pointset)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }
  file << std::fixed << std::setprecision(6);
  for (itk::PointSet<double, 3>::PointsContainer::Iterator iterator =
           pointset->GetPoints()->Begin();
       iterator < pointset->GetPoints()->End(); ++iterator)
    file << iterator->Value()[0] << " " << iterator->Value()[1] << " "
         << iterator->Value()[2] << "\n";
  file.close();
}

template <typename PixelType>
double evaluate_mi_with_both_images(
    const info_solve_registration<PixelType> &info_registration,
    size_t bin_numbers)
{
  using RegistrationPixelType = PixelType;
  constexpr unsigned int Dimension = 3;
  using ImageType = itk::Image<PixelType, Dimension>;
  using ImageRegistrationType = itk::Image<RegistrationPixelType, Dimension>;
  using MetricType =
      itk::MattesMutualInformationImageToImageMetricv4<ImageRegistrationType,
                                                       ImageRegistrationType>;
  using InterpolatorType =
      itk::LinearInterpolateImageFunction<ImageRegistrationType, double>;
  using TransformType = itk::VersorRigid3DTransform<double>;

  typename InterpolatorType::Pointer interpolator_moving =
      InterpolatorType::New();
  typename InterpolatorType::Pointer interpolator_fixed =
      InterpolatorType::New();

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

  TransformType::ParametersType displacement(
      initialTransform->GetNumberOfParameters());
  displacement.Fill(0);
  try
  {
    metric->Initialize();
    metric->SetParameters(displacement);

    return metric->GetValue();
  }
  catch (...)
  {
    return 100.0;
  }
}

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "need to call function with 3 params\n";
    return 1;
  }

  auto image_reader_fixed = itk::ImageFileReader<itk::Image<float, 3>>::New();
  image_reader_fixed->SetFileName(argv[1]);
  update_ikt_filter(image_reader_fixed);
  auto [point_cloud_fixed, mask_fixed_image] = extract_significant_regions(image_reader_fixed->GetOutput(), ExtractionSurfaceInfo<true>{3, 0.95, "fixed", 5, 5});
  write_point_cloud(point_cloud_fixed, "fixed_point_blob.txt");
  std::cout << "wrote point cloud\n";

  auto image_reader_moving = itk::ImageFileReader<itk::Image<float, 3>>::New();
  image_reader_moving->SetFileName(argv[2]);
  update_ikt_filter(image_reader_moving);
  auto [point_cloud_moving, mask_moving_image] = extract_significant_regions(image_reader_moving->GetOutput(), ExtractionSurfaceInfo<true>{3, 0.8, "moving", 5, 5});
  write_point_cloud(point_cloud_moving, "moving_point_blob.txt");
  std::cout << "wrote point cloud\n";

  Eigen::Matrix<double, 4, 4> Timage_centroid_fixed = Eigen::Matrix<double, 4, 4>::Identity();
  Timage_centroid_fixed.block<3, 1>(0, 3) = point_cloud_fixed.rowwise().mean();

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
    auto rescaler =
        itk::RescaleIntensityImageFilter<itk::Image<float, 3>,
                                         itk::Image<float, 3>>::New();
    rescaler->SetInput(image);
    rescaler->SetOutputMinimum(0);
    rescaler->SetOutputMaximum(itk::NumericTraits<unsigned char>::max());
    update_ikt_filter(rescaler);
    itk::Image<float, 3>::Pointer output = rescaler->GetOutput();
    return output;
  };

  auto fixed = converter(image_reader_fixed->GetOutput());
  auto moving = converter(image_reader_moving->GetOutput());
  auto ordered_solutions = curan::image::extract_potential_solutions(point_cloud_fixed, point_cloud_moving, 3);

  std::ofstream myfile{"results_of_fullscale_optimization.csv"};
  myfile << "run,iterations,bins,sampling percentage,relative_scales,learning "
            "rate,relaxation,convergence window,piramid sizes,bluring "
            "sizes,best cost,total time,true_error\n";

  // Optimizer parameters
  constexpr size_t local_permut = 1;

  std::vector<size_t> bin_numbers{100};
  std::vector<double> percentage_numbers{0.4};
  std::vector<double> relative_scales{1000.0};
  std::vector<double> learning_rate{.1};
  std::vector<double> relaxation_factor{0.7};
  std::vector<size_t> optimization_iterations{500};
  std::vector<size_t> convergence_window_size{40};

  std::array<std::array<size_t, size_info>, local_permut> piramid_sizes{{{3, 2, 1}}};
  std::array<std::array<double, size_info>, local_permut> bluering_sizes{{{2, 0, 0}}};

  size_t total_permutations = optimization_iterations.size() *
                              bin_numbers.size() * percentage_numbers.size() *
                              relative_scales.size() * learning_rate.size() *
                              relaxation_factor.size() *
                              convergence_window_size.size() *
                              piramid_sizes.size() * bluering_sizes.size() *
                              ordered_solutions.size();

  std::cout << "total permutations: " << total_permutations << std::endl;

  std::vector<std::tuple<double, double, Eigen::Matrix<double, 4, 4>>> full_runs;

  auto transformed_mask_fixed_image = itk::ImageMaskSpatialObject<3>::New();
  transformed_mask_fixed_image->SetImage(mask_fixed_image);
  update_ikt_filter(transformed_mask_fixed_image);
  auto transformed_mask_moving_image = itk::ImageMaskSpatialObject<3>::New();
  transformed_mask_moving_image->SetImage(mask_moving_image);
  update_ikt_filter(transformed_mask_moving_image);

  {
    size_t total_runs = 0;
    for (const auto &[T_arun_estimated_transform, cost_arun] : ordered_solutions)
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
                        modify_image_with_transform<float>(Timage_centroid_fixed.inverse() * Timage_origin_fixed, fixed);
                        modify_image_with_transform<float>(Timage_centroid_fixed.inverse() * T_arun_estimated_transform * Timage_origin_moving, moving);

                        print_image_with_transform<float>(fixed, "fixed_" + std::to_string(total_runs)+".mha");
                        print_image_with_transform<float>(moving, "moving_" + std::to_string(total_runs)+".mha");

                        modify_image_with_transform<unsigned char>(Timage_centroid_fixed.inverse() * Timage_origin_fixed, mask_fixed_image);
                        modify_image_with_transform<unsigned char>(Timage_centroid_fixed.inverse() * T_arun_estimated_transform * Timage_origin_moving, mask_moving_image);

                        std::chrono::steady_clock::time_point begin =
                            std::chrono::steady_clock::now();
                        auto [cost, transformation, initial_transform] =
                            solve_registration(
                                info_solve_registration<float>{
                                    fixed, moving,
                                    transformed_mask_fixed_image,
                                    transformed_mask_moving_image,
                                    Eigen::Matrix<double, 4,
                                                  4>::Identity()},
                                RegistrationParameters{
                                    bin_n, rel_scale, learn_rate, percent_n,
                                    relax_factor, wind_size, iters,
                                    pira_size, blur_size});
                        std::chrono::steady_clock::time_point end =
                            std::chrono::steady_clock::now();
                        double duration =
                            std::chrono::duration_cast<
                                std::chrono::seconds>(end - begin)
                                .count();
                        double final_real_cost = get_error(Timage_centroid_fixed * transformation * Timage_centroid_fixed.inverse() * T_arun_estimated_transform).mean();

                        myfile << total_runs << "," << iters << ","
                               << bin_n << "," << percent_n << ","
                               << rel_scale << "," << learn_rate << ","
                               << relax_factor << "," << wind_size
                               << ", {";
                        for (const auto &val : pira_size)
                          myfile << val << " ";
                        myfile << "}, {";
                        for (const auto &val : blur_size)
                          myfile << val << " ";
                        myfile
                            << "}," << cost << "," << duration << ","
                            << final_real_cost
                            << std::endl;

                        full_runs.emplace_back(cost, final_real_cost, transformation);
                        std::printf(
                            "mi (%.2f %%)\n",
                            (total_runs / (double)total_permutations) *
                                100);
                        ++total_runs;
                      }
  }

  const double pi = std::atan(1) * 4;

  for (const auto &[mi_cost, real_cost, T_transformation_mi] : full_runs)
  {
    std::cout << "error" << real_cost << std::endl;
  }

  return 0;
}