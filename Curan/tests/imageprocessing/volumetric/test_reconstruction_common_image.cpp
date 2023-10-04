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
    itk::SmartPointer<RegistrationType> registration;

    void set_registration(itk::SmartPointer<RegistrationType> in_registration)
    {
        registration = in_registration;
    }

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
        std::cout << optimizer->GetCurrentPosition() << "\n";
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

std::tuple<double, TransformType::Pointer> solve_registration(ImageType::Pointer fixed_image, ImageType::Pointer moving_image, curan::renderable::Renderable *volume_moving)
{
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
    TransformInitializerType::Pointer initializer = TransformInitializerType::New();

    TransformType::Pointer transform = TransformType::New();

    initializer->SetTransform(transform);
    initializer->SetFixedImage(fixed_image);
    initializer->SetMovingImage(moving_image);
    initializer->InitializeTransform();
    transform->SetRotation(0,0,0);

    registration->SetInitialTransform(transform);
    registration->InPlaceOn();
    registration->SetFixedImage(fixed_image);
    registration->SetMovingImage(moving_image);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(
        transform->GetNumberOfParameters());
    const double translationScale = 1.0/1000;
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

    try{
        registration->Update();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        throw err;
    }

    return {optimizer->GetCurrentMetricValue(), transform};
}

int main(int argc, char **argv)
{

    constexpr size_t number_of_iterations = 5;

    auto movingImageReader = MovingImageReaderType::New();

    std::string dirName2{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
    movingImageReader->SetFileName(dirName2);

    try{
        movingImageReader->Update();
    } catch (...) {
        std::printf("Failed to read the Precious Phantom\n");
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = movingImageReader->GetOutput();
    float origin_fixed[3];
    origin_fixed[0] = 1.0;
    origin_fixed[1] = 1.0;
    origin_fixed[2] = 1.0;

    pointer2fixedimage->SetOrigin(origin_fixed);
    ImageType::Pointer pointer2movingimage = ImageType::New();
    DeepCopy<ImageType>(pointer2fixedimage,pointer2movingimage);

    itk::Matrix<double,3,3> mat;
    mat.SetIdentity();

    srand((unsigned int) time(0));

    Eigen::Vector3f axis_along_rotation = Eigen::Vector3f::Random();
    axis_along_rotation.normalize();
    Eigen::AngleAxis<float> random_rotation(0.4,axis_along_rotation );
    auto matrix_ramdom_rotation = random_rotation.matrix();

    Eigen::Vector3f random_translation = Eigen::Vector3f::Random();
    random_translation *= 100;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            mat(row,col) = matrix_ramdom_rotation(row,col);
    pointer2movingimage->SetDirection(mat);
    float origin[3];
    origin[0] = random_translation[0]+origin_fixed[0];
    origin[1] = random_translation[1]+origin_fixed[1];
    origin[2] = random_translation[2]+origin_fixed[2];
    pointer2movingimage->SetOrigin(origin);

    std::cout << "\tFixed image direction:\n" <<  pointer2fixedimage->GetDirection() << "\tFixed Image Origin:\n" << pointer2fixedimage->GetOrigin()  << "\n\tFixed Image Spacing:\n" << pointer2fixedimage->GetSpacing() << "\n\n";
    std::cout << "\tMoving image direction:\n" <<  pointer2movingimage->GetDirection() << "\tMoving Image Origin:\n" << pointer2movingimage->GetOrigin() << "\n\tMoving Image Spacing:\n" << pointer2movingimage->GetSpacing() << "\n\n";

    std::vector<std::tuple<double, TransformType::Pointer>> full_runs;
    for (size_t iteration = 0; iteration < number_of_iterations; std::printf("\nIteration %d\n",iteration),++iteration)
        full_runs.emplace_back(solve_registration(pointer2fixedimage, pointer2movingimage, nullptr));

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = 1000000000000000000;
    for (const auto &possible : full_runs)
    {
        if (minimum_val > std::get<0>(possible))
        {
            minimum_index = current_index;
            minimum_val = std::get<0>(possible);
        }
        std::printf("optimized value: %f\n", std::get<0>(possible));
        ++current_index;
    }

    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    TransformType::MatrixType matrix = finalTransform->GetMatrix();
    TransformType::OffsetType offset = finalTransform->GetOffset();

    std::cout << "\n\tOptimized Direction:\n" << matrix << "\tOptimized Origin:\n" << offset << "\n\n";

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

    std::ofstream output_file{"C:/Users/SURGROB7/registration_results.json"};
    output_file << registration_transformation;
    return 0;
}