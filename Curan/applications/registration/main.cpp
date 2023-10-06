#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
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

const double pi = std::atan(1) * 4;

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::Euler3DTransform<double>;
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

void updateBaseTexture3D(vsg::floatArray3D &image, ImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<ImageType, ImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    ImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<ImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        ImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

class CommandType : public itk::Command
{
public:
    using Self = CommandType;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandType() = default;

public:
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using OptimizerPointer = const OptimizerType *;
    curan::renderable::Volume *moving_pointer_to_volume = nullptr;
    Eigen::Matrix<double, 4, 4> moving_homogenenous;
    TransformType::Pointer initialTransform;

    void set_pointer(curan::renderable::Volume *in_moving_pointer_to_volume)
    {
        moving_pointer_to_volume = in_moving_pointer_to_volume;
    }

    void Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }
    void Execute(const itk::Object *object, const itk::EventObject &event) override
    {
        auto optimizer = static_cast<OptimizerPointer>(object);
        TransformType::Pointer transform = TransformType::New();
        auto vals = optimizer->GetCurrentPosition();
        // std::cout << vals << "\n";
        transform->SetParameters(optimizer->GetCurrentPosition());
        transform->SetFixedParameters(initialTransform->GetFixedParameters());
        TransformType::MatrixType matrix = transform->GetMatrix();
        TransformType::OffsetType offset = transform->GetOffset();

        Eigen::Matrix<double, 4, 4> estimated_unknown_transformation = Eigen::Matrix<double, 4, 4>::Identity();

        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                estimated_unknown_transformation(row, col) = matrix(row, col);

        estimated_unknown_transformation(0, 3) = offset[0];
        estimated_unknown_transformation(1, 3) = offset[1];
        estimated_unknown_transformation(2, 3) = offset[2];

        auto estimated_overlap = estimated_unknown_transformation.inverse() * moving_homogenenous;

        auto moving_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);

        for (size_t row = 0; row < 4; ++row)
            for (size_t col = 0; col < 4; ++col)
                moving_homogenenous_transformation(col, row) = estimated_overlap(row, col);

        moving_homogenenous_transformation(3,0) *= 1e-3;
        moving_homogenenous_transformation(3,1) *= 1e-3;
        moving_homogenenous_transformation(3,2) *= 1e-3;

        if(moving_pointer_to_volume)
            moving_pointer_to_volume->update_transform(moving_homogenenous_transformation);
    }
};

template <typename TImage>
void DeepCopy(typename TImage::Pointer input, typename TImage::Pointer output)
{
    output->SetRegions(input->GetLargestPossibleRegion());
    output->Allocate();
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->SetDirection(input->GetDirection());
    itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd())
    {
        outputIterator.Set(inputIterator.Get());
        ++inputIterator;
        ++outputIterator;
    }
}

std::tuple<double, TransformType::Pointer> solve_registration(ImageType::Pointer fixed_image, ImageType::Pointer no_copy_moving_image, curan::renderable::Volume *volume_moving, const Eigen::Matrix<double, 4, 4> &moving_homogenenous)
{
    auto moving_image = ImageType::New();
    DeepCopy<ImageType>(no_copy_moving_image, moving_image);

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    unsigned int numberOfBins = 50;

    metric->SetNumberOfHistogramBins(numberOfBins);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);

    using TransformInitializerType =
        itk::CenteredTransformInitializer<TransformType,
                                          ImageType,
                                          ImageType>;

    auto initialTransform = TransformType::New();
    TransformInitializerType::Pointer initializer =
        TransformInitializerType::New();
    initializer->SetTransform(initialTransform);
    initializer->SetFixedImage(fixed_image);
    initializer->SetMovingImage(moving_image);
    initializer->InitializeTransform();

    Eigen::Vector3f random_rotation = Eigen::Vector3f::Random();
    random_rotation *= 10;

    initialTransform->SetRotation(random_rotation[0], random_rotation[1], random_rotation[2]);

    registration->InPlaceOn();
    registration->SetFixedImage(fixed_image);
    registration->SetMovingImage(moving_image);
    registration->SetInitialTransform(initialTransform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        initialTransform->GetNumberOfParameters());
    constexpr double translationScale = 1.0 / 1000.0;
    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = translationScale;
    optimizerScales[4] = translationScale;
    optimizerScales[5] = translationScale;
    optimizer->SetScales(optimizerScales);
    optimizer->SetNumberOfIterations(2000);
    optimizer->SetLearningRate(4);
    optimizer->SetMinimumStepLength(0.001);
    optimizer->SetReturnBestParametersAndValue(true);
    itk::SizeValueType value{10};
    optimizer->SetConvergenceWindowSize(value);
    optimizer->SetRelaxationFactor(0.8);

    constexpr unsigned int numberOfLevels = 4;

    RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(numberOfLevels);
    shrinkFactorsPerLevel[0] = 4;
    shrinkFactorsPerLevel[1] = 3;
    shrinkFactorsPerLevel[2] = 2;
    shrinkFactorsPerLevel[3] = 1;

    RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(numberOfLevels);
    smoothingSigmasPerLevel[0] = 2;
    smoothingSigmasPerLevel[1] = 1;
    smoothingSigmasPerLevel[2] = 0;
    smoothingSigmasPerLevel[3] = 0;

    registration->SetNumberOfLevels(numberOfLevels);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    RegistrationType::MetricSamplingStrategyEnum samplingStrategy =
        RegistrationType::MetricSamplingStrategyEnum::RANDOM;

    double samplingPercentage = 0.01;

    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(samplingPercentage);
    registration->MetricSamplingReinitializeSeed(121213);

    ImageType::RegionType region = fixed_image->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();
    ImageType::SpacingType spacing = fixed_image->GetSpacing();

    CommandType::Pointer observer = CommandType::New();
    optimizer->AddObserver(itk::StartEvent(), observer);
    optimizer->AddObserver(itk::IterationEvent(), observer);
    optimizer->AddObserver(itk::EndEvent(), observer);
    observer->moving_homogenenous = moving_homogenenous;
    observer->initialTransform = initialTransform;
    observer->moving_pointer_to_volume = volume_moving;
    try
    {
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        throw err;
    }

    return {optimizer->GetCurrentMetricValue(), initialTransform};
}

int main(int argc, char **argv)
{
constexpr size_t number_of_iterations = 10;

  auto fixedImageReader = FixedImageReaderType::New();
  auto movingImageReader = MovingImageReaderType::New();

  std::string dirName{CURAN_COPIED_RESOURCE_PATH"/reconstruction_results.mha"};
  fixedImageReader->SetFileName(dirName);

  std::string dirName2{CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha"};
  movingImageReader->SetFileName(dirName2);

  try{
    fixedImageReader->Update();
    movingImageReader->Update();
  } catch (...) {
    std::string error_name = "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n"+std::string(CURAN_COPIED_RESOURCE_PATH);
    std::printf(error_name.c_str());
    return 1;
  }

  ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
  ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

    Eigen::Matrix<double, 4, 4> mat_moving_here = Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            mat_moving_here(row, col) = pointer2movingimage->GetDirection()(row, col);

    mat_moving_here(0, 3) = pointer2movingimage->GetOrigin()[0];
    mat_moving_here(1, 3) = pointer2movingimage->GetOrigin()[1];
    mat_moving_here(2, 3) = pointer2movingimage->GetOrigin()[2];

    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

    ImageType::RegionType region_fixed = pointer2fixedimage->GetLargestPossibleRegion();
    ImageType::SizeType size_itk_fixed = region_fixed.GetSize();
    ImageType::SpacingType spacing_fixed = pointer2fixedimage->GetSpacing();

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size_itk_fixed.GetSize()[0];
    volumeinfo.height = size_itk_fixed.GetSize()[1];
    volumeinfo.depth = size_itk_fixed.GetSize()[2];
    volumeinfo.spacing_x = spacing_fixed[0];
    volumeinfo.spacing_y = spacing_fixed[1];
    volumeinfo.spacing_z = spacing_fixed[2];

    auto volume_fixed = curan::renderable::Volume::make(volumeinfo);
    window << volume_fixed;

    auto direction = pointer2fixedimage->GetDirection();
    auto origin = pointer2fixedimage->GetOrigin();
    auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            fixed_homogenenous_transformation(col, row) = direction(row, col);

    volume_fixed->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

    auto casted_volume_fixed = volume_fixed->cast<curan::renderable::Volume>();
    auto updater = [pointer2fixedimage](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer2fixedimage); };
    casted_volume_fixed->update_volume(updater);

    ImageType::RegionType region_moving = pointer2movingimage->GetLargestPossibleRegion();
    ImageType::SizeType size_itk_moving = region_moving.GetSize();
    ImageType::SpacingType spacing_moving = pointer2movingimage->GetSpacing();

    volumeinfo.width = size_itk_moving.GetSize()[0];
    volumeinfo.height = size_itk_moving.GetSize()[1];
    volumeinfo.depth = size_itk_moving.GetSize()[2];
    volumeinfo.spacing_x = spacing_moving[0];
    volumeinfo.spacing_y = spacing_moving[1];
    volumeinfo.spacing_z = spacing_moving[2];

    auto volume_moving = curan::renderable::Volume::make(volumeinfo);
    window << volume_moving;

    auto casted_volume_moving = volume_moving->cast<curan::renderable::Volume>();
    auto updater_moving = [pointer2movingimage](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer2movingimage); };

    casted_volume_moving->update_volume(updater_moving);

    auto moving_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    moving_homogenenous_transformation(3, 0) = pointer2movingimage->GetOrigin()[0] / 1000.0;
    moving_homogenenous_transformation(3, 1) = pointer2movingimage->GetOrigin()[1] / 1000.0;
    moving_homogenenous_transformation(3, 2) = pointer2movingimage->GetOrigin()[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            moving_homogenenous_transformation(col, row) = pointer2movingimage->GetDirection()(row, col);

    casted_volume_moving->update_transform(moving_homogenenous_transformation);

    std::vector<std::tuple<double, TransformType::Pointer>> full_runs;

    std::thread run_registration_algorithm{[&](){
        for (size_t iteration = 0; iteration < number_of_iterations; std::printf("\nIteration %d\n", iteration), ++iteration)
            full_runs.emplace_back(solve_registration(pointer2fixedimage, pointer2movingimage, casted_volume_moving, mat_moving_here));
    }};

    window.run();
    run_registration_algorithm.join();

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = 1e20;
    for (const auto &possible : full_runs)
    {
        if (minimum_val > std::get<0>(possible))
        {
            minimum_index = current_index;
            minimum_val = std::get<0>(possible);
        }
        ++current_index;
    }

    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            moving_homogenenous_transformation(col, row) = finalTransform->GetMatrix()(row, col);
    moving_homogenenous_transformation(3,0) = finalTransform->GetOffset()[0];
    moving_homogenenous_transformation(3,1) = finalTransform->GetOffset()[1];
    moving_homogenenous_transformation(3,2) = finalTransform->GetOffset()[2];
    casted_volume_moving->update_transform(moving_homogenenous_transformation);

    auto origin_fixed_mine = pointer2fixedimage->GetOrigin();

    TransformType::MatrixType matrix = finalTransform->GetMatrix();
    TransformType::OffsetType offset = finalTransform->GetOffset();

    std::stringstream matrix_value;
    for (size_t y = 0; y < 3; ++y)
    {
        for (size_t x = 0; x < 3; ++x)
        {
            float matrix_entry = matrix[x][y];
            matrix_value << matrix_entry << " ";
        }
        matrix_value << "\n ";
    }

    nlohmann::json registration_transformation;
    registration_transformation["Matrix"] = matrix_value.str();
    registration_transformation["Offset"] = offset;

    std::ofstream output_file{CURAN_COPIED_RESOURCE_PATH "/registration_results.json"};
    output_file << registration_transformation;
    return 0;
}