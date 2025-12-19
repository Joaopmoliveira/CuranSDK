#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/DicomDisplay.h"
#include "userinterface/widgets/ColorDicomDisplay.h"
#include "userinterface/widgets/MiniPage.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/ItemExplorer.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/MutatingTextPanel.h"
#include <unordered_map>
#include <optional>
#include <charconv>
#include <functional>
#include "utils/Job.h"
#include "utils/TheadPool.h"
#include "utils/FileStructures.h"
#include "utils/DateManipulation.h"

#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "userinterface/widgets/ImageWrapper.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkImageFileWriter.h"
#include "itkOrientImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageSeriesReader.h"
#include "itkRayCastInterpolateImageFunction.h"

#include <Eigen/Dense>
#include "itkCheckerBoardImageFilter.h"

using DicomPixelType = std::uint16_t;
constexpr unsigned int Dimension = 3;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

#include "itkVersorRigid3DTransform.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkEuler3DTransform.h"
#include "itkCenteredTransformInitializer.h"

template <typename TImage>
typename TImage::Pointer DeepCopy(typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    for(;!inputIterator.IsAtEnd(); ++inputIterator,++outputIterator ){
        outputIterator.Set(inputIterator.Get());
    }
    return output;
}

struct ColorCoding{
    std::array<float,2> ct_limits = {0.2650f,0.3175f};
    std::array<float,2> mri_limits = {0.2150f,0.3725f};
};

template <typename TImageOut, typename TImage>
typename TImageOut::Pointer DeepColoredCopy(typename TImage::Pointer input, const std::array<float,2>& color_coding,std::array<float,3> target_color)
{
    typename TImageOut::Pointer output = TImageOut::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImageOut> outputIterator(output, output->GetLargestPossibleRegion());

    for(;!inputIterator.IsAtEnd(); ++inputIterator,++outputIterator ){
        auto pixel = inputIterator.Get();
        typename TImageOut::PixelType tocopy;
        if(pixel > 255*color_coding[0] && pixel < 255*color_coding[1]){
            tocopy[0] = target_color[0];
            tocopy[1] = target_color[1];
            tocopy[2] = target_color[2];
        } else {
            tocopy[0] = pixel;
            tocopy[1] = pixel;
            tocopy[2] = pixel;
        }
        outputIterator.Set(tocopy);
    }
    return output;
}

template <typename TImage,typename InclusionPolicy>
std::tuple<typename TImage::Pointer,itk::Image<unsigned char,3>::Pointer> DeepCopyWithInclusionPolicy(InclusionPolicy&& inclusion_policy,typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    auto mask = itk::Image<unsigned char,3>::New();
    mask->SetRegions(input->GetLargestPossibleRegion());
    mask->SetDirection(input->GetDirection());
    mask->SetSpacing(input->GetSpacing());
    mask->SetOrigin(input->GetOrigin());
    mask->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());
    itk::ImageRegionIterator<itk::Image<unsigned char,3>> maskIterator(mask, mask->GetLargestPossibleRegion());

    for(;!inputIterator.IsAtEnd(); ++inputIterator,++outputIterator,++maskIterator ){
        if(inclusion_policy(inputIterator.GetIndex(),input)){
            outputIterator.Set(inputIterator.Get());
            maskIterator.Set(255);
        }else{
            outputIterator.Set(0);
            maskIterator.Set(0);
        }
    }
    return {output,mask};
}

struct ACPCData{
    bool ac_specification = false;
    curan::ui::Button* ac_button = nullptr;
    std::optional<Eigen::Matrix<double,3,1>> ac_word_coordinates;
    bool pc_specification = false;
    curan::ui::Button* pc_button = nullptr;
    std::optional<Eigen::Matrix<double,3,1>> pc_word_coordinates;
};

struct TrajectoryConeData{
    bool target_specification = false;
    curan::ui::Button* target_button = nullptr;
    std::optional<Eigen::Matrix<double,3,1>> target_world_coordinates;
    bool main_diagonal_specification = false;
    curan::ui::Button* main_diagonal_button = nullptr;
    std::optional<Eigen::Matrix<double,3,1>> main_diagonal_word_coordinates;
    bool entry_specification = false;
    std::optional<Eigen::Matrix<double,3,1>> entry_point_word_coordinates;
    curan::geometry::Piramid piramid_world_coordinates;
};

struct RegionOfInterest{
    std::vector<curan::ui::directed_stroke> paths;
    bool is_selecting_path = false;
};

struct CachedVolume{
   DICOMImageType::Pointer img;
    bool is_visible = true;
};

struct ColoredCachedVolume{
    curan::ui::ColorDicomViewer::ImageType::Pointer img;
    bool is_visible = true;
};

enum LayoutType{
    ONE,
    TWO,
    THREE
};

enum ViewType{
    CT_VIEW,
    MRI_VIEW
};



struct Application;

struct Application{
    curan::ui::MiniPage* tradable_page = nullptr;
    ACPCData ac_pc_data;
    TrajectoryConeData trajectory_location;
    LayoutType type = LayoutType::THREE;
    ViewType modalitytype = ViewType::CT_VIEW;
    std::map<std::string,CachedVolume> high_resolution_ct_volumes;
    std::map<std::string,CachedVolume> high_resolution_mri_volumes;
    std::map<std::string,ColoredCachedVolume> miscellaneous_colored_volumes;
    std::map<std::string,CachedVolume> miscellaneous_volumes;
    std::map<std::string,CachedVolume> ct_volumes;
    std::map<std::string,CachedVolume> mri_volumes;
    std::string current_volume = "raw";
    Eigen::Matrix<double,4,4> registration_fixed_to_moving = Eigen::Matrix<double,4,4>::Identity();
    curan::ui::DicomVolumetricMask<std::uint16_t>* vol_mas = nullptr;
    curan::ui::IconResources* resources = nullptr;
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(2);
    std::function<std::unique_ptr<curan::ui::Container>(Application&)> panel_constructor;
    std::function<void(Application&,curan::ui::DicomVolumetricMask<std::uint16_t>*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> volume_callback;
    curan::ui::ColorDicomVolumetricMask color_vol_mas{nullptr};
    std::function<void(Application&,curan::ui::ColorDicomVolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> projected_volume_callback;
    RegionOfInterest roi;
    curan::utilities::SafeQueue<curan::utilities::Job> sync_tasks_with_screen_queue;
    ColorCoding color_coding;
    //we need to read these when doing the registration pipeline. The point is that the optimizer runs, queries the current location of the slices in the fixed volume
    //computes the intersection with the reoriented image, and then updates the overlays
    std::mutex mut;
    std::vector<curan::ui::DicomViewer<std::uint16_t>*> viewers;
    bool is_update_in_progress = false;
    float TransparencyValue = 0.204961;
    float AlphaFuncValue = 0.052590;
    float SampleDensityValue = 0.1;

    Application(curan::ui::IconResources & in_resources,curan::ui::DicomVolumetricMask<std::uint16_t>* in_vol_mas): resources{&in_resources},vol_mas{in_vol_mas}{}

    std::unique_ptr<curan::ui::Container> main_page();
};

enum Strategy{
    CONSERVATIVE,   
};

std::unique_ptr<curan::ui::Overlay> layout_overlay(Application& appdata);
std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata);
std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page(Application& appdata,int mask);
std::unique_ptr<curan::ui::Overlay> create_slider_range_page(Application& appdata);
std::unique_ptr<curan::ui::Overlay> create_projection_page(Application& appdata);

//these functions are declared by the order that they follow during the planning procedure
std::unique_ptr<curan::ui::Container> select_registration_mri_ct(Application& appdata);

void ac_pc_midline_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_ac_pc_midline(Application& appdata);

std::unique_ptr<curan::ui::Container> validate_color_coding(Application& appdata);

void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_target_and_region_of_entry(Application& appdata);

void select_entry_point_and_validate(Application& appdata,curan::ui::ColorDicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_entry_point_and_validate_point_selection(Application& appdata);

void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_roi_for_surgery(Application& appdata);

template <typename TRegistration,typename TransformType>
class RegistrationInterfaceCommand : public itk::Command {
public:
    using Self = RegistrationInterfaceCommand;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

    double m_LastMetricValue;
    Application* ptr = nullptr;
    DICOMImageType::Pointer f_fixed;
    DICOMImageType::Pointer f_moving;
    TransformType::Pointer transform;
    float g_rotation_angle = 0.0;
protected:
    RegistrationInterfaceCommand() { m_LastMetricValue = 0.0; };

public:
    using RegistrationType = TRegistration;
    using RegistrationPointer = RegistrationType *;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;

    void Execute(itk::Object * caller, const itk::EventObject & event) override   {
        Execute((const itk::Object *)caller, event);
    }

    void Execute(const itk::Object * object, const itk::EventObject & event) override
    {
        auto optimizer = static_cast<const OptimizerType *>(object);
        if(!optimizer) return;

        double currentValue = optimizer->GetValue();
        // Only print out when the Metric value changes
        if (itk::Math::abs(m_LastMetricValue - currentValue) > 1e-7)
        {
            g_rotation_angle += 10;
            m_LastMetricValue = currentValue;

            if(ptr){
                std::lock_guard<std::mutex> g{ptr->mut};
                if(ptr->is_update_in_progress)
                    return;
                ptr->is_update_in_progress = true;
                for(auto viewer : ptr->viewers){
                    // now I need to query for the current size of the dicom viewer (notice that this entire code only works because the resampled image is exactly on top of the physical image)
                    auto fixed_slice_physical = viewer->physical_viewed_image();
                    using FilterType = itk::ResampleImageFilter<DICOMImageType, DICOMImageType>;
                    auto resample = FilterType::New();
                    using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
                    resample->SetInput(f_moving);
                    resample->SetTransform(transform);
                    resample->SetSize(fixed_slice_physical->GetLargestPossibleRegion().GetSize());
                    resample->SetOutputOrigin(fixed_slice_physical->GetOrigin());
                    resample->SetOutputSpacing(fixed_slice_physical->GetSpacing());
                    resample->SetOutputDirection(fixed_slice_physical->GetDirection());
                    resample->SetDefaultPixelValue(100);
                    
                    using CastFilterType = itk::CastImageFilter<DICOMImageType,DICOMImageType>;
                    
                    auto caster = CastFilterType::New();
                    caster->SetInput(resample->GetOutput());

                    try{
                        caster->Update();
                    } catch(...){
                        std::printf("failed to resample volume");
                        return;
                    }

                    auto buff = curan::utilities::CaptureBuffer::make_shared(caster->GetOutput()->GetBufferPointer(),caster->GetOutput()->GetPixelContainer()->Size()*sizeof(char),caster->GetOutput());
                    auto extracted_size = caster->GetOutput()->GetLargestPossibleRegion().GetSize();
                    auto rotation = g_rotation_angle;
                    switch (viewer->get_direction())
                    {
                    case curan::ui::Direction::X:
                    {
                        curan::ui::ImageWrapper wrapper = curan::ui::ImageWrapper{buff, extracted_size[1], extracted_size[2]};
                        
                        viewer->update_custom_drawingcall([=](SkCanvas* canvas, SkRect image_rec, SkRect widget_rec){
                            const std::array<SkColor,5> colors = {SK_ColorGREEN, SK_ColorYELLOW, SK_ColorRED, SK_ColorBLUE, SK_ColorGREEN};
                            const SkScalar pos[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
                            sk_sp<SkShader> shader = SkGradientShader::MakeSweep(
                                image_rec.centerX(), image_rec.centerY(), colors.data(), pos, colors.size()
                            );
                            SkMatrix matrix;
                            matrix.setRotate(rotation, image_rec.centerX(), image_rec.centerY()); 
                            sk_sp<SkShader> animated_shader = shader->makeWithLocalMatrix(matrix);
                            SkPaint paint_square;
                            paint_square.setStyle(SkPaint::kStroke_Style);
                            paint_square.setAntiAlias(true);
                            paint_square.setStrokeWidth(4);
                            paint_square.setShader(animated_shader);
                            canvas->drawRect(image_rec,paint_square);
                            SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
                            SkPaint paintBlend;
                            paintBlend.setAlpha(128);
                            canvas->drawImageRect(wrapper.image, image_rec, opt,&paintBlend);
                        });
                        break;
                    }
                    case curan::ui::Direction::Y:
                    {
                        curan::ui::ImageWrapper wrapper = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[2]};
                        viewer->update_custom_drawingcall([=](SkCanvas* canvas, SkRect image_rec, SkRect widget_rec){
                            const std::array<SkColor,5> colors = {SK_ColorGREEN, SK_ColorYELLOW, SK_ColorRED, SK_ColorBLUE, SK_ColorGREEN};
                            const SkScalar pos[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
                            sk_sp<SkShader> shader = SkGradientShader::MakeSweep(
                                image_rec.centerX(), image_rec.centerY(), colors.data(), pos, colors.size()
                            );
                            SkMatrix matrix;
                            matrix.setRotate(rotation, image_rec.centerX(), image_rec.centerY()); 
                            sk_sp<SkShader> animated_shader = shader->makeWithLocalMatrix(matrix);
                            SkPaint paint_square;
                            paint_square.setStyle(SkPaint::kStroke_Style);
                            paint_square.setAntiAlias(true);
                            paint_square.setStrokeWidth(4);
                            paint_square.setShader(animated_shader);
                            canvas->drawRect(image_rec,paint_square);
                            SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
                            SkPaint paintBlend;
                            paintBlend.setAlpha(128);
                            canvas->drawImageRect(wrapper.image, image_rec, opt,&paintBlend);
                        });
                        break;
                    }
                    case curan::ui::Direction::Z:
                    {
                        curan::ui::ImageWrapper wrapper = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[1]};
                        viewer->update_custom_drawingcall([=](SkCanvas* canvas, SkRect image_rec, SkRect widget_rec){
                            const std::array<SkColor,5> colors = {SK_ColorGREEN, SK_ColorYELLOW, SK_ColorRED, SK_ColorBLUE, SK_ColorGREEN};
                            const SkScalar pos[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
                            sk_sp<SkShader> shader = SkGradientShader::MakeSweep(
                                image_rec.centerX(), image_rec.centerY(), colors.data(), pos, colors.size()
                            );
                            SkMatrix matrix;
                            matrix.setRotate(rotation, image_rec.centerX(), image_rec.centerY()); 
                            sk_sp<SkShader> animated_shader = shader->makeWithLocalMatrix(matrix);
                            SkPaint paint_square;
                            paint_square.setStyle(SkPaint::kStroke_Style);
                            paint_square.setAntiAlias(true);
                            paint_square.setStrokeWidth(4);
                            paint_square.setShader(animated_shader);
                            canvas->drawRect(image_rec,paint_square);
                            SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
                            SkPaint paintBlend;
                            paintBlend.setAlpha(128);
                            canvas->drawImageRect(wrapper.image, image_rec, opt,&paintBlend);
                        });
                        break;
                    }
                    }
                }
                ptr->is_update_in_progress = false;
            }
        }
    }


    void set(DICOMImageType::Pointer fixed,DICOMImageType::Pointer moving,Application& appdata,TransformType::Pointer intransform){
        f_fixed = fixed;
        f_moving = moving;
        ptr = &appdata;
        transform = intransform;
    };

};

std::tuple<DICOMImageType::Pointer,DICOMImageType::Pointer> solve_registration(DICOMImageType::Pointer fixed_image,DICOMImageType::Pointer moving_image, Application& appdata) {
    using ImageRegistrationType = DicomPixelType;
    using TransformType = itk::VersorRigid3DTransform<double>;
    using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using MetricType = itk::MattesMutualInformationImageToImageMetricv4<DICOMImageType,DICOMImageType>;
    using RegistrationType =  itk::ImageRegistrationMethodv4<DICOMImageType,DICOMImageType,TransformType>;

    auto metric = MetricType::New();
    auto optimizer = OptimizerType::New();
    auto registration = RegistrationType::New();
    typename InterpolatorType::Pointer interpolator_moving = InterpolatorType::New();
    typename InterpolatorType::Pointer interpolator_fixed = InterpolatorType::New();

    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);

    metric->SetNumberOfHistogramBins(24);

    metric->SetUseMovingImageGradientFilter(false);
    metric->SetUseFixedImageGradientFilter(false);

    metric->SetMovingInterpolator(interpolator_moving);
    metric->SetFixedInterpolator(interpolator_fixed);

    auto transform = TransformType::New();

    using TransformInitializerType = itk::CenteredTransformInitializer<TransformType,DICOMImageType,DICOMImageType>;
    auto initializer = TransformInitializerType::New();
    initializer->SetTransform(transform);
    initializer->SetFixedImage(fixed_image);
    initializer->SetMovingImage(moving_image);
    initializer->MomentsOn();
    initializer->InitializeTransform();

    registration->SetFixedImage(fixed_image);
    registration->SetMovingImage(moving_image);
    registration->SetInitialTransform(transform);

    using OptimizerScalesType = OptimizerType::ScalesType;
    OptimizerScalesType optimizerScales(transform->GetNumberOfParameters());

    optimizerScales[0] = 1.0;
    optimizerScales[1] = 1.0;
    optimizerScales[2] = 1.0;
    optimizerScales[3] = 1.0 / 1000;
    optimizerScales[4] = 1.0 / 1000;
    optimizerScales[5] = 1.0 / 1000;

    optimizer->SetScales(optimizerScales);

    optimizer->SetNumberOfIterations(300);

    optimizer->SetLearningRate(5.0);
    optimizer->SetMinimumStepLength(0.0001);
    optimizer->SetReturnBestParametersAndValue(true);
    optimizer->SetRelaxationFactor(0.5);

    typename RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    constexpr size_t cascaded_dim = 2; 
    shrinkFactorsPerLevel.SetSize(cascaded_dim);
    typename RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(cascaded_dim);
    std::array<size_t,cascaded_dim> piramid_sizes{4,1};
    std::array<size_t,cascaded_dim> bluering_sizes{2,0};
    for (size_t i = 0; i < cascaded_dim; ++i) {
        shrinkFactorsPerLevel[i] = piramid_sizes[i];
        smoothingSigmasPerLevel[i] = bluering_sizes[i];
    }

    registration->SetNumberOfLevels(cascaded_dim);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(0.2);


    registration->InPlaceOn();

    using CommandType = RegistrationInterfaceCommand<RegistrationType,TransformType>;
    auto command = CommandType::New();
    optimizer->AddObserver(itk::IterationEvent(), command);
    command->set(fixed_image,moving_image,appdata,transform);

    try {
        registration->Update();
    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return std::make_tuple(DICOMImageType::Pointer(),DICOMImageType::Pointer());
    }

    {
        std::lock_guard<std::mutex> g{appdata.mut};
        for(auto viewer : appdata.viewers){
            viewer->clear_custom_drawingcall();
        }
    }

    using ResampleFilterType = itk::ResampleImageFilter<DICOMImageType, DICOMImageType>;
    auto finalTransform = registration->GetOutput()->Get();
    auto resample = ResampleFilterType::New();
    
    resample->SetTransform(finalTransform);
    resample->SetInput(moving_image);
    
    resample->SetSize(fixed_image->GetLargestPossibleRegion().GetSize());
    resample->SetOutputOrigin(fixed_image->GetOrigin());
    resample->SetOutputSpacing(fixed_image->GetSpacing());
    resample->SetOutputDirection(fixed_image->GetDirection());
    resample->SetDefaultPixelValue(0);
    
    using CastFilterType = itk::CastImageFilter<DICOMImageType,DICOMImageType>;
    
    auto caster = CastFilterType::New();
    caster->SetInput(resample->GetOutput());

    try {
        caster->Update();
    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return std::make_tuple(DICOMImageType::Pointer(),DICOMImageType::Pointer());
    }

    
   DICOMImageType::Pointer resampled_output = caster->GetOutput();
    
    using CheckerBoardFilterType = itk::CheckerBoardImageFilter<DICOMImageType>;
    
    auto checker = CheckerBoardFilterType::New();
    
    checker->SetInput1(fixed_image);
    checker->SetInput2(resample->GetOutput());
    caster = CastFilterType::New();
    caster->SetInput(checker->GetOutput());
    try {
        caster->Update();
    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return std::make_tuple(DICOMImageType::Pointer(),DICOMImageType::Pointer());
    }

   DICOMImageType::Pointer checked_overlap_output = caster->GetOutput();
    return std::make_tuple(resampled_output,checked_overlap_output);
}

curan::ui::ColorDicomViewer::ImageType::Pointer allocate_image_itk_based(
    Application& appdata, 
   DICOMImageType::Pointer input , 
    const std::array<float,2>& color_coding, 
    const std::array<float,3>& colortoreplace,
    float TransparencyValue,
    float AlphaFuncValue,
    float SampleDensityValue) {
    auto convert_to_eigen = [&](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
        Eigen::Matrix<double,3,1> converted;
        converted << (double)point[0] , (double)point[1], (double)point[2];
        return converted;
    };

    auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
    auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);
    auto base2 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[3]);
    auto base3 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[4]);

    Eigen::Matrix<double,3,1> x_direction = (base1 - base0).normalized();
    Eigen::Matrix<double,3,1> y_direction = (base3 - base0).normalized();
    Eigen::Matrix<double, 3, 1> z_direction = x_direction.cross(y_direction);
    
    Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
    eigen_rotation_matrix.col(0) = x_direction;
    eigen_rotation_matrix.col(1) = y_direction;
    eigen_rotation_matrix.col(2) = z_direction;

    curan::ui::ColorDicomViewer::ImageType::Pointer projectionimage = curan::ui::ColorDicomViewer::ImageType::New();
  
    curan::ui::ColorDicomViewer::ImageType::SizeType size;
    size[0] = 512;  // size along X
    size[1] = 512;  // size along Y 
    size[2] = 1;   // size along Z

    curan::ui::ColorDicomViewer::ImageType::SpacingType spacing;
    spacing[0] = (base1 - base0).norm()/(double)size[0]; // mm along X
    spacing[1] = (base3 - base0).norm()/(double)size[1]; // mm along X
    spacing[2] = 1.0; // mm along Z
  
    curan::ui::ColorDicomViewer::ImageType::PointType origin;
    origin[0] = base0[0];
    origin[1] = base0[1];
    origin[2] = base0[2];
  
    auto direction = input->GetDirection();
    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            direction(i,j) = eigen_rotation_matrix(i,j);
  
    curan::ui::ColorDicomViewer::ImageType::RegionType region;
    region.SetSize(size);
  
    projectionimage->SetRegions(region);
    projectionimage->SetSpacing(spacing);
    projectionimage->SetOrigin(origin);
    projectionimage->SetDirection(direction);
    projectionimage->Allocate();
    curan::ui::ColorDicomViewer::ImageType::PixelType rgb;
    rgb[0] = 0;
    rgb[1] = 0;
    rgb[2] = 0;
    projectionimage->FillBuffer(rgb);

    auto mix = [](Eigen::Matrix<double,4,1> l, Eigen::Matrix<double,4,1> r,double mix_ratio){
        Eigen::Matrix<double,3,1> mixed;
        mixed = l.block<3,1>(0,0)*(1.0-mix_ratio)+r.block<3,1>(0,0)*mix_ratio;
        return mixed;
    };

    // World-space ray source
    const Eigen::Vector3d t0_world = *appdata.trajectory_location.target_world_coordinates;
    using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
    auto interpolator = InterpolatorType::New();
    interpolator->SetInputImage(input);

    // 2. Pre-calculate the ray start in index space
    itk::ContinuousIndex<double, 3> startIdx;
    curan::ui::ColorDicomViewer::ImageType::PointType t0_itk;
    t0_itk[0] = t0_world[0]; t0_itk[1] = t0_world[1]; t0_itk[2] = t0_world[2];
    input->TransformPhysicalPointToContinuousIndex(t0_itk, startIdx);
    float initial_alpha = (1.0/255.0) * interpolator->EvaluateAtContinuousIndex(startIdx);

    // 2. Multi-thread your loop
    itk::MultiThreaderBase::Pointer multiThreader = itk::MultiThreaderBase::New();
    multiThreader->ParallelizeImageRegion<3>(
        projectionimage->GetRequestedRegion(),
        [&](const curan::ui::ColorDicomViewer::ImageType::RegionType& region) {
            
            itk::ImageRegionIteratorWithIndex<curan::ui::ColorDicomViewer::ImageType> it(projectionimage, region);
            it.GoToBegin();
            curan::ui::ColorDicomViewer::ImageType::PointType t0_itk;
            t0_itk[0] = t0_world[0]; t0_itk[1] = t0_world[1]; t0_itk[2] = t0_world[2];
            
            for (; !it.IsAtEnd(); ++it) {
                size_t number_of_accumulations = 0;
                curan::ui::ColorDicomViewer::ImageType::PointType te_phys;
                projectionimage->TransformIndexToPhysicalPoint(it.GetIndex(), te_phys);
                itk::ContinuousIndex<double, 3> endIdx;
                input->TransformPhysicalPointToContinuousIndex(te_phys, endIdx);  

                Eigen::Matrix<double,4,1> accumulatedColor{0, 0, 0, 0};

                auto dirIdx = endIdx - startIdx;
                double distIdx = std::sqrt(dirIdx[0]*dirIdx[0] + dirIdx[1]*dirIdx[1] + dirIdx[2]*dirIdx[2]);
                auto stepIdx = (dirIdx / distIdx) * 0.5;
                int numSteps = static_cast<int>(distIdx / 0.5);
                double offset = static_cast<double>(std::rand()) / RAND_MAX;
                itk::ContinuousIndex<double, 3> currentIdx = startIdx;
                currentIdx += stepIdx * offset;
                for (int s = 0; s < numSteps; ++s) {
                    if (!interpolator->IsInsideBuffer(currentIdx)) {
                                        currentIdx += stepIdx;
                                        continue;
                    }

                    float pixelVal = (1.0/255.0)* interpolator->EvaluateAtContinuousIndex(currentIdx);

                    Eigen::Matrix<double,4,1> src{pixelVal, pixelVal,pixelVal, pixelVal * TransparencyValue};
                    if(pixelVal > color_coding[0] && pixelVal < color_coding[1]) {
                        src[0] = colortoreplace[0];
                        src[1] = colortoreplace[1];
                        src[2] = colortoreplace[2];
                    }

                    float current_alpha = src[3];
                    if (current_alpha > AlphaFuncValue) {
                        double weight = (1.0 - accumulatedColor[3]) * current_alpha;
                        accumulatedColor.head<3>() += weight * src.head<3>();
                        accumulatedColor[3] += weight;
                        ++number_of_accumulations;
                    }

                    if (accumulatedColor[3] >= 0.98) break;

                    currentIdx += stepIdx;
                }
                //std::printf("number_of_accumulations: %llu\n",number_of_accumulations);
                curan::ui::ColorDicomViewer::ImageType::PixelType pix;
                pix[0] = 255.0*accumulatedColor[0];
                pix[1] = 255.0*accumulatedColor[1];
                pix[2] = 255.0*accumulatedColor[2];
                it.Set(pix);
            }
        }, nullptr);

    return projectionimage;
}

curan::ui::ColorDicomViewer::ImageType::Pointer allocate_image_itk_based_phong(
    Application& appdata, 
   DICOMImageType::Pointer input , 
    const std::array<float,2>& color_coding, 
    const std::array<float,3>& colortoreplace,
    float TransparencyValue,
    float AlphaFuncValue,
    float SampleDensityValue) {
    auto convert_to_eigen = [&](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
        Eigen::Matrix<double,3,1> converted;
        converted << (double)point[0] , (double)point[1], (double)point[2];
        return converted;
    };

    auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
    auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);
    auto base2 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[3]);
    auto base3 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[4]);

    Eigen::Matrix<double,3,1> x_direction = (base1 - base0).normalized();
    Eigen::Matrix<double,3,1> y_direction = (base3 - base0).normalized();
    Eigen::Matrix<double, 3, 1> z_direction = x_direction.cross(y_direction);
    
    Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
    eigen_rotation_matrix.col(0) = x_direction;
    eigen_rotation_matrix.col(1) = y_direction;
    eigen_rotation_matrix.col(2) = z_direction;

    curan::ui::ColorDicomViewer::ImageType::Pointer projectionimage = curan::ui::ColorDicomViewer::ImageType::New();
  
    curan::ui::ColorDicomViewer::ImageType::SizeType size;
    size[0] = 512;  // size along X
    size[1] = 512;  // size along Y 
    size[2] = 1;   // size along Z

    curan::ui::ColorDicomViewer::ImageType::SpacingType spacing;
    spacing[0] = (base1 - base0).norm()/(double)size[0]; // mm along X
    spacing[1] = (base3 - base0).norm()/(double)size[1]; // mm along X
    spacing[2] = 1.0; // mm along Z
  
    curan::ui::ColorDicomViewer::ImageType::PointType origin;
    origin[0] = base0[0];
    origin[1] = base0[1];
    origin[2] = base0[2];
  
    auto direction = input->GetDirection();
    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            direction(i,j) = eigen_rotation_matrix(i,j);
  
    curan::ui::ColorDicomViewer::ImageType::RegionType region;
    region.SetSize(size);
  
    projectionimage->SetRegions(region);
    projectionimage->SetSpacing(spacing);
    projectionimage->SetOrigin(origin);
    projectionimage->SetDirection(direction);
    projectionimage->Allocate();
    curan::ui::ColorDicomViewer::ImageType::PixelType rgb;
    rgb[0] = 0;
    rgb[1] = 0;
    rgb[2] = 0;
    projectionimage->FillBuffer(rgb);

    auto mix = [](Eigen::Matrix<double,4,1> l, Eigen::Matrix<double,4,1> r,double mix_ratio){
        Eigen::Matrix<double,3,1> mixed;
        mixed = l.block<3,1>(0,0)*(1.0-mix_ratio)+r.block<3,1>(0,0)*mix_ratio;
        return mixed;
    };

    // World-space ray source
    const Eigen::Vector3d t0_world = *appdata.trajectory_location.target_world_coordinates;
    using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
    auto interpolator = InterpolatorType::New();
    interpolator->SetInputImage(input);

    using GradientCalculatorType = itk::CentralDifferenceImageFunction<DICOMImageType, double>;
    auto gradientCalculator = GradientCalculatorType::New();
    gradientCalculator->SetInputImage(input);

    // 2. Pre-calculate the ray start in index space
    itk::ContinuousIndex<double, 3> startIdx;
    curan::ui::ColorDicomViewer::ImageType::PointType t0_itk;
    t0_itk[0] = t0_world[0]; t0_itk[1] = t0_world[1]; t0_itk[2] = t0_world[2];
    input->TransformPhysicalPointToContinuousIndex(t0_itk, startIdx);
    float initial_alpha = (1.0/255.0) * interpolator->EvaluateAtContinuousIndex(startIdx);

    // 2. Multi-thread your loop
    itk::MultiThreaderBase::Pointer multiThreader = itk::MultiThreaderBase::New();
    multiThreader->ParallelizeImageRegion<3>(
        projectionimage->GetRequestedRegion(),
        [&](const curan::ui::ColorDicomViewer::ImageType::RegionType& region) {
            
            itk::ImageRegionIteratorWithIndex<curan::ui::ColorDicomViewer::ImageType> it(projectionimage, region);
            it.GoToBegin();
            curan::ui::ColorDicomViewer::ImageType::PointType t0_itk;
            t0_itk[0] = t0_world[0]; t0_itk[1] = t0_world[1]; t0_itk[2] = t0_world[2];
            
            for (; !it.IsAtEnd(); ++it) {
                size_t number_of_accumulations = 0;
                curan::ui::ColorDicomViewer::ImageType::PointType te_phys;
                projectionimage->TransformIndexToPhysicalPoint(it.GetIndex(), te_phys);
                Eigen::Vector3d te_world;
                te_world[0] = te_phys[0];
                te_world[1] = te_phys[1];
                te_world[2] = te_phys[2];
                itk::ContinuousIndex<double, 3> endIdx;
                input->TransformPhysicalPointToContinuousIndex(te_phys, endIdx);  

                Eigen::Matrix<double,4,1> accumulatedColor{0, 0, 0, 0};

                Eigen::Vector3d lightDirWorld = (t0_world - te_world).normalized();

                auto dirIdx = endIdx - startIdx;
                double distIdx = std::sqrt(dirIdx[0]*dirIdx[0] + dirIdx[1]*dirIdx[1] + dirIdx[2]*dirIdx[2]);
                auto stepIdx = (dirIdx / distIdx) * 0.5;
                int numSteps = static_cast<int>(distIdx / 0.5);
                double offset = static_cast<double>(std::rand()) / RAND_MAX;
                itk::ContinuousIndex<double, 3> currentIdx = startIdx;
                currentIdx += stepIdx * offset;
                for (int s = 0; s < numSteps; ++s) {
                    if (!interpolator->IsInsideBuffer(currentIdx)) {
                                        currentIdx += stepIdx;
                                        continue;
                    }

                    float pixelVal = (1.0/255.0)* interpolator->EvaluateAtContinuousIndex(currentIdx);
                    auto gradITK = gradientCalculator->EvaluateAtContinuousIndex(currentIdx);
                    Eigen::Vector3d normal(gradITK[0], gradITK[1], gradITK[2]);

                    double shading = 1.0;    
                    double gradMagnitude = normal.norm();
                    if (gradMagnitude > 0.05) { 
                        normal /= gradMagnitude; // Normalize
                        double diffuse = std::max(0.0, -normal.dot(lightDirWorld)); 
                        shading = 0.2 + 0.8 * diffuse;
                    }

                    Eigen::Matrix<double,4,1> src{pixelVal*shading, pixelVal*shading,pixelVal*shading, pixelVal * TransparencyValue};
                    if(pixelVal > color_coding[0] && pixelVal < color_coding[1]) {
                        src[0] = colortoreplace[0]*shading;
                        src[1] = colortoreplace[1]*shading;
                        src[2] = colortoreplace[2]*shading;
                    }

                    float current_alpha = src[3];
                    if (current_alpha > AlphaFuncValue) {
                        double weight = (1.0 - accumulatedColor[3]) * current_alpha;
                        accumulatedColor.head<3>() += weight * src.head<3>();
                        accumulatedColor[3] += weight;
                        ++number_of_accumulations;
                    }

                    if (accumulatedColor[3] >= 0.98) break;

                    currentIdx += stepIdx;
                }
                //std::printf("number_of_accumulations: %llu\n",number_of_accumulations);
                curan::ui::ColorDicomViewer::ImageType::PixelType pix;
                pix[0] = 255.0*accumulatedColor[0];
                pix[1] = 255.0*accumulatedColor[1];
                pix[2] = 255.0*accumulatedColor[2];
                it.Set(pix);
            }
        }, nullptr);

    return projectionimage;
}

struct BoundingBox{

    enum FrameOrientation{
        RIGHT_HANDED,
        LEFT_HANDED
    };

    Eigen::Matrix<double,3,1> origin;
    Eigen::Matrix<double,3,3> orientation;
    Eigen::Matrix<double,3,1> size;
    Eigen::Matrix<double,3,1> spacing;
    FrameOrientation frameorientation = FrameOrientation::RIGHT_HANDED;

    BoundingBox(const Eigen::Matrix<double,3,1>& in_origin,const Eigen::Matrix<double,3,1>& along_x,Eigen::Matrix<double,3,1> along_y,Eigen::Matrix<double,3,1> along_z, Eigen::Matrix<double,3,1> in_spacing){
        origin = in_origin;
        Eigen::Matrix<double,3,1> direct_x = along_x-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_x = direct_x;
        Eigen::Matrix<double,3,1> direct_y = along_y-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_y = direct_y;
        Eigen::Matrix<double,3,1> direct_z = along_z-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_z = direct_z;
        direct_x.normalize();
        direct_y.normalize();
        direct_z.normalize();
        orientation.col(0) = direct_x;
        orientation.col(1) = direct_y;
        orientation.col(2) = direct_z;

        double determinant = orientation.determinant();
        if(determinant< 0.9999 || determinant>1.0001)
            frameorientation = FrameOrientation::RIGHT_HANDED;
        else if (-determinant< 0.9999 || -determinant>1.0001)
            frameorientation = FrameOrientation::LEFT_HANDED;
        else 
            throw std::runtime_error("failure to generate an ortogonal rotation matrix");

        spacing = in_spacing;
        size[0] = vector_along_direction_x.norm()/spacing[0];
        size[1] = vector_along_direction_y.norm()/spacing[1];
        size[2] = vector_along_direction_z.norm()/spacing[2];
    }

    BoundingBox(DICOMImageType::Pointer image){
        Eigen::Matrix<double, 3, 1> origin_for_bounding_box{{image->GetOrigin()[0], image->GetOrigin()[1], image->GetOrigin()[2]}};
       DICOMImageType::PointType itk_along_dimension_x;
       DICOMImageType::IndexType index_along_x{{(long long)image->GetLargestPossibleRegion().GetSize()[0], 0, 0}};
        image->TransformIndexToPhysicalPoint(index_along_x, itk_along_dimension_x);
        Eigen::Matrix<double, 3, 1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0], itk_along_dimension_x[1], itk_along_dimension_x[2]}};
       DICOMImageType::PointType itk_along_dimension_y;
       DICOMImageType::IndexType index_along_y{{0, (long long)image->GetLargestPossibleRegion().GetSize()[1], 0}};
        image->TransformIndexToPhysicalPoint(index_along_y, itk_along_dimension_y);
        Eigen::Matrix<double, 3, 1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0], itk_along_dimension_y[1], itk_along_dimension_y[2]}};
       DICOMImageType::PointType itk_along_dimension_z;
       DICOMImageType::IndexType index_along_z{{0, 0, (long long)image->GetLargestPossibleRegion().GetSize()[2]}};
        image->TransformIndexToPhysicalPoint(index_along_z, itk_along_dimension_z);
        Eigen::Matrix<double, 3, 1> extrema_along_z_for_bounding_box{{itk_along_dimension_z[0], itk_along_dimension_z[1], itk_along_dimension_z[2]}};
        Eigen::Matrix<double, 3, 1> in_spacing{{image->GetSpacing()[0], image->GetSpacing()[1], image->GetSpacing()[2]}};

        Eigen::Matrix<double,3,1> in_origin = origin_for_bounding_box;
        Eigen::Matrix<double,3,1> along_x = extrema_along_x_for_bounding_box;
        Eigen::Matrix<double,3,1> along_y = extrema_along_y_for_bounding_box;
        Eigen::Matrix<double,3,1> along_z = extrema_along_z_for_bounding_box;
        
        origin = in_origin;
        Eigen::Matrix<double,3,1> direct_x = along_x-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_x = direct_x;
        Eigen::Matrix<double,3,1> direct_y = along_y-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_y = direct_y;
        Eigen::Matrix<double,3,1> direct_z = along_z-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_z = direct_z;
        direct_x.normalize();
        direct_y.normalize();
        direct_z.normalize();
        orientation.col(0) = direct_x;
        orientation.col(1) = direct_y;
        orientation.col(2) = direct_z;

        double determinant = orientation.determinant();
        if(determinant< 0.9999 || determinant>1.0001)
            frameorientation = FrameOrientation::RIGHT_HANDED;
        else if (-determinant< 0.9999 || -determinant>1.0001)
            frameorientation = FrameOrientation::LEFT_HANDED;
        else 
            throw std::runtime_error("failure to generate an ortogonal rotation matrix");

        spacing = in_spacing;
        size[0] = vector_along_direction_x.norm()/spacing[0];
        size[1] = vector_along_direction_y.norm()/spacing[1];
        size[2] = vector_along_direction_z.norm()/spacing[2];
    }

    BoundingBox(const BoundingBox& other) : origin{other.origin},orientation{other.orientation},size{other.size},spacing{other.spacing}{};
    
    BoundingBox centered_bounding_box(const Eigen::Matrix<double,3,3>& desired_orientation){

        //if(debug) std::cout << "\ndebug info: (relative_transform)\n" <<  relative_transform;
        
        Eigen::Matrix<double,3,8> corners_in_rotated_space;
        corners_in_rotated_space.col(0)[0] = 0;
        corners_in_rotated_space.col(0)[1] = 0;
        corners_in_rotated_space.col(0)[2] = 0;

        corners_in_rotated_space.col(1)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(1)[1] = 0;
        corners_in_rotated_space.col(1)[2] = 0;

        corners_in_rotated_space.col(2)[0] = 0;
        corners_in_rotated_space.col(2)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(2)[2] = 0;

        corners_in_rotated_space.col(3)[0] = 0;
        corners_in_rotated_space.col(3)[1] = 0;
        corners_in_rotated_space.col(3)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(4)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(4)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(4)[2] = 0;

        corners_in_rotated_space.col(5)[0] = 0;
        corners_in_rotated_space.col(5)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(5)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(6)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(6)[1] = 0;
        corners_in_rotated_space.col(6)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(7)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(7)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(7)[2] = spacing[2]*size[2];

        //if(debug) std::cout << "\ndebug info: (corners_in_rotated_space)\n" <<  corners_in_rotated_space;

        Eigen::Matrix<double,3,8> transformed_corners_in_world_space;
        for(size_t col = 0; col < static_cast<size_t>(transformed_corners_in_world_space.cols()); ++col)
            transformed_corners_in_world_space.col(col) = orientation*corners_in_rotated_space.col(col)+origin;

        Eigen::Matrix<double,3,8> transformed_corners_aligned_with_desired_orientation;
        for(size_t col = 0; col < static_cast<size_t>(transformed_corners_in_world_space.cols()); ++col)
            transformed_corners_aligned_with_desired_orientation.col(col) = desired_orientation.transpose()*transformed_corners_in_world_space.col(col);
            
        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> minimum = transformed_corners_aligned_with_desired_orientation.rowwise().minCoeff();

        //if(debug) std::cout << "\ndebug info: (minimum)\n" <<  transformed_corners_in_rotated_space.rowwise().minCoeff();

        transformed_corners_aligned_with_desired_orientation.colwise() -=minimum;

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> unrounded_transformed_size = transformed_corners_aligned_with_desired_orientation.rowwise().maxCoeff();

        //if(debug) std::cout << "\ndebug info: (unrounded_transformed_size)\n" <<  unrounded_transformed_size;

        Eigen::Matrix<double,3,1> transformed_spacing = spacing;
        transformed_spacing.fill(spacing.minCoeff());

        //if(debug) std::cout << "\ndebug info: (transformed_spacing)\n" <<  transformed_spacing;

        Eigen::Matrix<double,3,1> transformed_size;
        transformed_size[0] = std::ceil(unrounded_transformed_size[0]/transformed_spacing[0]);
        transformed_size[1] = std::ceil(unrounded_transformed_size[1]/transformed_spacing[1]);
        transformed_size[2] = std::ceil(unrounded_transformed_size[2]/transformed_spacing[2]);

        transformed_spacing[0] = unrounded_transformed_size[0]/transformed_size[0];
        transformed_spacing[1] = unrounded_transformed_size[1]/transformed_size[1];
        transformed_spacing[2] = unrounded_transformed_size[2]/transformed_size[2];

        //if(debug) std::cout << "\ndebug info: (transformed_size)\n" <<  transformed_size;

        //if(debug) std::cout << "\ndebug info: (transformed_spacing)\n" <<  transformed_spacing;

        Eigen::Matrix<double,3,4> transformed_corners_in_pixel_space;
        transformed_corners_in_pixel_space.col(0)[0] = 0;
        transformed_corners_in_pixel_space.col(0)[1] = 0;
        transformed_corners_in_pixel_space.col(0)[2] = 0;

        transformed_corners_in_pixel_space.col(1)[0] = transformed_spacing[0]*transformed_size[0];
        transformed_corners_in_pixel_space.col(1)[1] = 0;
        transformed_corners_in_pixel_space.col(1)[2] = 0;

        transformed_corners_in_pixel_space.col(2)[0] = 0;
        transformed_corners_in_pixel_space.col(2)[1] = transformed_spacing[1]*transformed_size[1];
        transformed_corners_in_pixel_space.col(2)[2] = 0;

        transformed_corners_in_pixel_space.col(3)[0] = 0;
        transformed_corners_in_pixel_space.col(3)[1] = 0;
        transformed_corners_in_pixel_space.col(3)[2] = transformed_spacing[2]*transformed_size[2];

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_pixel_space)\n" <<  transformed_corners_in_pixel_space;

        auto quatered_bounding_box = orientation*(1.0/2.0)*corners_in_rotated_space;
        //if(debug) std::cout << "\ndebug info: (quatered_bounding_box)\n" <<  quatered_bounding_box;
        Eigen::Matrix<double,3,1> center_bounding_box = quatered_bounding_box.col(1)+quatered_bounding_box.col(2)+quatered_bounding_box.col(3);
        //if(debug) std::cout << "\ndebug info: (center_bounding_box)\n" <<  center_bounding_box;

        Eigen::Matrix<double,3,3> transformed_rotation;

        transformed_rotation = desired_orientation;

        auto transformed_quatered_bounding_box = desired_orientation*(1.0/2.0)*transformed_corners_in_pixel_space;
        //if(debug) std::cout << "\ndebug info: (transformed_quatered_bounding_box)\n" <<  transformed_quatered_bounding_box;
        Eigen::Matrix<double,3,1> transformed_center_bounding_box = transformed_quatered_bounding_box.col(1)+transformed_quatered_bounding_box.col(2)+transformed_quatered_bounding_box.col(3);
        //if(debug) std::cout << "\ndebug info: (transformed_center_bounding_box)\n" <<  transformed_center_bounding_box;
        Eigen::Matrix<double,3,4> transformed_corners_of_rotated_box_in_world_space;
        transformed_corners_of_rotated_box_in_world_space.col(0) = origin+center_bounding_box-transformed_center_bounding_box;
        transformed_corners_of_rotated_box_in_world_space.col(1) = transformed_corners_of_rotated_box_in_world_space.col(0)+desired_orientation*transformed_corners_in_pixel_space.col(1);
        transformed_corners_of_rotated_box_in_world_space.col(2) = transformed_corners_of_rotated_box_in_world_space.col(0)+desired_orientation*transformed_corners_in_pixel_space.col(2);
        transformed_corners_of_rotated_box_in_world_space.col(3) = transformed_corners_of_rotated_box_in_world_space.col(0)+desired_orientation*transformed_corners_in_pixel_space.col(3);

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_world_space)\n" <<  transformed_corners_in_world_space;

        return BoundingBox{transformed_corners_of_rotated_box_in_world_space.col(0),transformed_corners_of_rotated_box_in_world_space.col(1),transformed_corners_of_rotated_box_in_world_space.col(2),transformed_corners_of_rotated_box_in_world_space.col(3),transformed_spacing};
    }
};


std::unique_ptr<curan::ui::Overlay> layout_overlay(Application& appdata)
{
    using namespace curan::ui;
    auto single_view_layout = Button::make(" ", "layout1x1.png", *appdata.resources);
    single_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    single_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { 
        appdata.type = LayoutType::ONE; 
        {
            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers = std::vector<curan::ui::DicomViewer<std::uint16_t>*>{}; 
        }
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); 
    });

    auto double_view_layout = Button::make(" ", "layout1x2.png", *appdata.resources);
    double_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    double_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { 
        appdata.type = LayoutType::TWO; 
        {
            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers = std::vector<curan::ui::DicomViewer<std::uint16_t>*>{}; 
        }
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); 
    });

    auto triple_view_layout = Button::make(" ", "layout1x3.png", *appdata.resources);
    triple_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    triple_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { 
        appdata.type = LayoutType::THREE; 
        {
            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers = std::vector<curan::ui::DicomViewer<std::uint16_t>*>{}; 
        }
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); 
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(single_view_layout) << std::move(double_view_layout) << std::move(triple_view_layout);
    viwers_container->set_color(SK_ColorTRANSPARENT);
    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata){
    using namespace curan::ui;
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    auto overlay_lambda = [&](DicomViewer<std::uint16_t>* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
        switch(selected_option){
            case 0:
                viewer->update_volume(appdata.vol_mas, Direction::X);
            break;
            case 1:
                viewer->update_volume(appdata.vol_mas, Direction::Y);
            break;
            case 2:
                viewer->update_volume(appdata.vol_mas, Direction::Z);
            break;
            case 3:
                viewer->change_zoom();
            break;
            case 4:
                viewer->change_path_state(PathState::HIGHLIGHTPATH);
            break;
            case 5:
                viewer->change_path_state(PathState::DELETEPATH);
                break;
            case 6:
                viewer->change_path_state(PathState::DRAWPATH);
                break;
            default:
            break;
        }
    };
    switch(appdata.type){
        case ONE:
        {
            auto image_display = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::X);
            if(appdata.current_volume.compare("raw")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_display->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_display->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }
            image_display->add_overlay_processor(overlay_lambda);
            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers.push_back(image_display.get());
            *container << std::move(image_display);
        }
        break;
        case TWO:
        {
            auto image_displayx = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::X);
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            image_displayy->add_overlay_processor(overlay_lambda);

            if(appdata.current_volume.compare("raw")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }

            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers.push_back(image_displayx.get());
            appdata.viewers.push_back(image_displayy.get());
            *container << std::move(image_displayx) << std::move(image_displayy);
        }
        break;
        case THREE:
        {
            auto image_displayx = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::X);
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            image_displayy->add_overlay_processor(overlay_lambda);
            auto image_displayz = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::Z);
            image_displayz->add_overlay_processor(overlay_lambda);

            if(appdata.current_volume.compare("raw")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }

            std::lock_guard<std::mutex> g{appdata.mut};
            appdata.viewers.push_back(image_displayx.get());
            appdata.viewers.push_back(image_displayy.get());
            appdata.viewers.push_back(image_displayz.get());
            *container << std::move(image_displayx) << std::move(image_displayy) << std::move(image_displayz);
        }
        break;
        default:
        throw std::runtime_error("cannot process layout type");
    }
    return std::move(container);
}

std::unique_ptr<curan::ui::Container> create_colored_dicom_viewers(Application& appdata){
    using namespace curan::ui;
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    auto overlay_lambda = [&](ColorDicomViewer* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
        switch(selected_option){
            case 0:
                viewer->update_volume(&appdata.color_vol_mas, Direction::X);
            break;
            case 1:
                viewer->update_volume(&appdata.color_vol_mas, Direction::Y);
            break;
            case 2:
                viewer->update_volume(&appdata.color_vol_mas, Direction::Z);
            break;
            case 3:
                viewer->change_zoom();
            break;
            case 4:
                viewer->change_path_state(ColorPathState::COLORHIGHLIGHTPATH);
            break;
            case 5:
                viewer->change_path_state(ColorPathState::COLORDELETEPATH);
                break;
            case 6:
                viewer->change_path_state(ColorPathState::COLORDRAWPATH);
                break;
            default:
            break;
        }
    };
    switch(appdata.type){
        case ONE:
        {
            auto image_display = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::X);
            if(appdata.current_volume.compare("raw")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_display->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_display->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_display->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }
            image_display->add_overlay_processor(overlay_lambda);
            *container << std::move(image_display);
        }
        break;
        case TWO:
        {
            auto image_displayx = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::X);
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::Y);
            image_displayy->add_overlay_processor(overlay_lambda);

            if(appdata.current_volume.compare("raw")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }
            *container << std::move(image_displayx) << std::move(image_displayy);
        }
        break;
        case THREE:
        {
            auto image_displayx = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::X);
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::Y);
            image_displayy->add_overlay_processor(overlay_lambda);
            auto image_displayz = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::Z);
            image_displayz->add_overlay_processor(overlay_lambda);

            if(appdata.current_volume.compare("raw")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("source")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("acpc")==0){
                image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("alongtrajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            } else if(appdata.current_volume.compare("trajectory")==0){
                image_displayx->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayy->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
                image_displayz->push_options({"view 1","view 2","bird eye","zoom","select path","delete path","draw path"});
            }
            *container << std::move(image_displayx) << std::move(image_displayy) << std::move(image_displayz);
        }
        break;
        default:
        throw std::runtime_error("cannot process layout type");
    }
    return std::move(container);
}

std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning,curan::ui::IconResources& resources)
{
    using namespace curan::ui;
    auto warn = Button::make(" ", "warning.png", resources);
    warn->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(400, 200));

    auto button = Button::make(warning, resources);
    button->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(200, 50));

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *viwers_container << std::move(warn) << std::move(button);
    viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0, .8, 1.0});

    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success,curan::ui::IconResources& resources)
{
    using namespace curan::ui;
    auto warn = Button::make(" ", "submit.png", resources);
    warn->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(400, 200));

    auto button = Button::make(success, resources);
    button->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(200, 50));

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *viwers_container << std::move(warn) << std::move(button);
    viwers_container->set_color(SK_ColorTRANSPARENT)
        .set_divisions({0.0, .8, 1.0});

    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page(Application& appdata, int mask)
{
    using namespace curan::ui;
    auto item_explorer = ItemExplorer::make("file_icon.png", *appdata.resources);
    item_explorer->add_press_call([&,mask](ItemExplorer *widget, Press press, ConfigDraw *draw){
            auto highlighted = widget->highlighted();
            assert(highlighted.size()==1 && "the size is larger than one");
            size_t volume_index = highlighted.back();
            
            size_t i = 0;

            if(appdata.modalitytype == ViewType::CT_VIEW){
                for(auto& [volume_description,cached_entry] : appdata.ct_volumes){
                    if(cached_entry.is_visible){
                        if(i==volume_index){
                            appdata.vol_mas->update_volume(cached_entry.img,mask);
                            appdata.current_volume = volume_description;
                        }
                        ++i;
                    }
                }
            } else {
                for(auto& [volume_description,cached_entry] : appdata.mri_volumes){
                    if(cached_entry.is_visible){
                        if(i==volume_index ){
                            appdata.vol_mas->update_volume(cached_entry.img,mask);
                            appdata.current_volume = volume_description;
                        }
                        ++i;
                    }
                }
            }

            std::printf("new volume index is: %s\n",appdata.current_volume.c_str());

    });
    using ExtractFilterType = itk::ExtractImageFilter<DICOMImageType,DICOMImageType>;
    size_t identifier = 0;
    auto volumes = &appdata.ct_volumes;
    if(appdata.modalitytype != ViewType::CT_VIEW)
        volumes = &appdata.mri_volumes;
    for (auto &[description,cached_volume] : *volumes)
    {
        if(cached_volume.is_visible){
            if (cached_volume.img.IsNotNull())
            {
                auto itk_pointer = cached_volume.img;
                auto extract_filter = ExtractFilterType::New();
                extract_filter->SetDirectionCollapseToSubmatrix();
                extract_filter->SetInput(itk_pointer);

               DICOMImageType::RegionType inputRegion = itk_pointer->GetBufferedRegion();
               DICOMImageType::SpacingType spacing = itk_pointer->GetSpacing();
               DICOMImageType::SizeType size = inputRegion.GetSize();

                auto copy_size = size;
                size[Direction::Z] = 1;

               DICOMImageType::IndexType start = inputRegion.GetIndex();
                start[Direction::Z] = std::floor(copy_size[Direction::Z] / 2.0);
               DICOMImageType::RegionType desiredRegion;
                desiredRegion.SetSize(size);
                desiredRegion.SetIndex(start);
                extract_filter->SetExtractionRegion(desiredRegion);
                extract_filter->UpdateLargestPossibleRegion();

               DICOMImageType::Pointer pointer_to_block_of_memory = extract_filter->GetOutput();
               DICOMImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
                auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(DicomPixelType), pointer_to_block_of_memory);
                auto extracted_size = pointer_to_block_of_memory->GetBufferedRegion().GetSize();
                item_explorer->add(Item{identifier, description, buff, extracted_size[0], extracted_size[1]});
            }
            ++identifier;
        }
    }
    item_explorer->set_size(SkRect::MakeWH(800, 400));
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(item_explorer);
    return Overlay::make(std::move(container), SkColorSetARGB(100, 125, 125, 125), true);
}


std::unique_ptr<curan::ui::Overlay> create_slider_range_page(Application& appdata)
{
    using namespace curan::ui;
	auto slider = Slider::make({ 0.0f, 255.0f });
    float high_value = appdata.modalitytype == ViewType::CT_VIEW ? appdata.color_coding.ct_limits[1] : appdata.color_coding.mri_limits[1];
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_current_value(high_value).set_size(SkRect::MakeWH(400, 40));
	auto textblob = TextBlob::make("higher range");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 60));
	slider->set_callback([&](Slider* slider, ConfigDraw* config) {
        if(appdata.modalitytype == ViewType::CT_VIEW){
            auto higher_ct_value = slider->get_current_value();
            if(higher_ct_value < appdata.color_coding.ct_limits[0])
                 slider->set_current_value(appdata.color_coding.ct_limits[0]+0.01);
            else
                appdata.color_coding.ct_limits[1] = higher_ct_value;

            if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end()){
                auto vol =  ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.ct_limits,{255.0f,0.0f,0.0f})};
                appdata.color_vol_mas.update_volume(vol.img);
                appdata.miscellaneous_colored_volumes["ct_source"] = vol;
            }else{
                config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
                return;
            }

        } else {
            auto higher_mri_value = slider->get_current_value();
            if(higher_mri_value < appdata.color_coding.mri_limits[0])
                 slider->set_current_value(appdata.color_coding.mri_limits[0]+0.01);
            else
                appdata.color_coding.mri_limits[1] = higher_mri_value;

            if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end()){
                auto vol = ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.mri_limits,{0.0f,0.0f,255.0f})};
                appdata.color_vol_mas.update_volume(vol.img);
                appdata.miscellaneous_colored_volumes["mri_source"] = vol;
            }else{
                config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
                return;
            }

        }
	});

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(textblob) << std::move(slider);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });

    float low_value = appdata.modalitytype == ViewType::CT_VIEW ? appdata.color_coding.ct_limits[0] : appdata.color_coding.mri_limits[0];
	auto slider1 = Slider::make({ 0.0f, 255.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_current_value(low_value).set_size(SkRect::MakeWH(400, 40));
	auto textblob1 = TextBlob::make("lower range");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 60));
	slider1->set_callback([&](Slider* slider, ConfigDraw* config) {
        if(appdata.modalitytype == ViewType::CT_VIEW){
            auto lower_ct_value = slider->get_current_value();
            if(lower_ct_value > appdata.color_coding.ct_limits[1])
                 slider->set_current_value(appdata.color_coding.ct_limits[1]-0.01);
            else
                appdata.color_coding.ct_limits[0] = lower_ct_value;

            if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end()){
                auto vol =  ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.ct_limits,{255.0f,0.0f,0.0f})};
                appdata.color_vol_mas.update_volume(vol.img);
                appdata.miscellaneous_colored_volumes["ct_source"] = vol;
            }else{
                config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
                return;
            }

        } else {
            auto lower_mri_value = slider->get_current_value();
            if(lower_mri_value > appdata.color_coding.mri_limits[1])
                 slider->set_current_value(appdata.color_coding.mri_limits[1]-0.01);
            else
                appdata.color_coding.mri_limits[0] = lower_mri_value;

            if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end()){
                auto vol = ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.mri_limits,{0.0f,0.0f,255.0f})};
                appdata.color_vol_mas.update_volume(vol.img);
                appdata.miscellaneous_colored_volumes["mri_source"] = vol;
            }else{
                config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
                return;
            }
        }
	});

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(textblob1) << std::move(slider1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });

    auto containerofcontainers = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *containerofcontainers << std::move(container) << std::move(container1);
    return Overlay::make(std::move(containerofcontainers), SkColorSetARGB(100, 125, 125, 125), true);
}



std::unique_ptr<curan::ui::Overlay> create_projection_page(Application& appdata)
{
    using namespace curan::ui;
	auto slider = Slider::make({ 0.0001f, 0.5f });
    auto range = slider->get_limits();
    std::printf("range[%.2f %.2f]\n",range[1],range[0]);
    float transparency_value =  (appdata.TransparencyValue-range[0])/(range[1]-range[0]);
    std::printf("value[%.2f]\n",transparency_value);
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_current_value(transparency_value).set_size(SkRect::MakeWH(400, 40));
	auto textblob = TextBlob::make("transparency");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 60));
	slider->set_callback([&](Slider* slider, ConfigDraw* config) {
        auto transparency = slider->get_current_value();
        auto range = slider->get_limits();
        appdata.TransparencyValue = transparency*(range[1]-range[0])+range[0];
        //std::printf("appdata.TransparencyValue[%.6f] appdata.AlphaFuncValue[%.6f]\n",appdata.TransparencyValue,appdata.AlphaFuncValue);
        try{                                                                                                                                                                    
            if(appdata.modalitytype == ViewType::CT_VIEW){
               DICOMImageType::Pointer ct_input;
                if (auto search = appdata.ct_volumes.find("trajectory"); search != appdata.ct_volumes.end())
                    ct_input = search->second.img;
                else
                    throw std::runtime_error("failure due to missing volume");
                curan::ui::ColorDicomViewer::ImageType::Pointer ct_projected_input = allocate_image_itk_based_phong(appdata,ct_input,appdata.color_coding.ct_limits,{1.0f,0.0f,0.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
                appdata.miscellaneous_colored_volumes["ct_projection"] = ColoredCachedVolume{ct_projected_input};
                appdata.color_vol_mas.update_volume(ct_projected_input);
            }
            else {
               DICOMImageType::Pointer mri_input;
                if (auto search = appdata.mri_volumes.find("trajectory"); search != appdata.mri_volumes.end())
                    mri_input = search->second.img;
                else
                throw std::runtime_error("failure due to missing volume");

                curan::ui::ColorDicomViewer::ImageType::Pointer mri_projected_input = allocate_image_itk_based_phong(appdata,mri_input,appdata.color_coding.mri_limits,{0.0f,0.0f,1.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
                appdata.miscellaneous_colored_volumes["mri_projection"] = ColoredCachedVolume{mri_projected_input};
                appdata.color_vol_mas.update_volume(mri_projected_input);
            }
        } catch(...){
            std::cout << "failure allocating image" << std::endl;
            throw std::runtime_error("failure");
        }
	});

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(textblob) << std::move(slider);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });

    float low_value = appdata.modalitytype == ViewType::CT_VIEW ? appdata.color_coding.ct_limits[0] : appdata.color_coding.mri_limits[0];
	auto slider1 = Slider::make({ 0.0001f, 0.5f }); 
    range = slider1->get_limits();
    std::printf("range[%.2f %.2f]\n",range[1],range[0]);
    float alpha_value =  (appdata.AlphaFuncValue-range[0])/(range[1]-range[0]);
    std::printf("value[%.2f]\n",alpha_value);
    slider1->set_current_value((appdata.AlphaFuncValue-range[0])/(range[1]-range[0]));
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorDKGRAY).set_current_value(alpha_value).set_size(SkRect::MakeWH(400, 40));
	auto textblob1 = TextBlob::make("alpha");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 60));
	slider1->set_callback([&](Slider* slider, ConfigDraw* config) {
        auto alpha = slider->get_current_value();
        auto range = slider->get_limits();
        appdata.AlphaFuncValue = alpha*(range[1]-range[0])+range[0];
        //std::printf("appdata.TransparencyValue[%.6f] appdata.AlphaFuncValue[%.6f]\n",appdata.TransparencyValue,appdata.AlphaFuncValue);
        try{                                                                                                                                                                    
            if(appdata.modalitytype == ViewType::CT_VIEW){
               DICOMImageType::Pointer ct_input;
                if (auto search = appdata.ct_volumes.find("trajectory"); search != appdata.ct_volumes.end())
                    ct_input = search->second.img;
                else
                    throw std::runtime_error("failure due to missing volume");
                curan::ui::ColorDicomViewer::ImageType::Pointer ct_projected_input = allocate_image_itk_based_phong(appdata,ct_input,appdata.color_coding.ct_limits,{1.0f,0.0f,0.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
                appdata.miscellaneous_colored_volumes["ct_projection"] = ColoredCachedVolume{ct_projected_input};
                appdata.color_vol_mas.update_volume(ct_projected_input);
            }
            else {
               DICOMImageType::Pointer mri_input;
                if (auto search = appdata.mri_volumes.find("trajectory"); search != appdata.mri_volumes.end())
                    mri_input = search->second.img;
                else
                throw std::runtime_error("failure due to missing volume");

                curan::ui::ColorDicomViewer::ImageType::Pointer mri_projected_input = allocate_image_itk_based_phong(appdata,mri_input,appdata.color_coding.mri_limits,{0.0f,0.0f,1.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
                appdata.miscellaneous_colored_volumes["mri_projection"] = ColoredCachedVolume{mri_projected_input};
                appdata.color_vol_mas.update_volume(mri_projected_input);
            }
            std::printf("added projection!\n");
        } catch(...){
            std::cout << "failure allocating image" << std::endl;
            throw std::runtime_error("failure");
        }
	});

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(textblob1) << std::move(slider1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });

    auto containerofcontainers = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *containerofcontainers << std::move(container) << std::move(container1);
    return Overlay::make(std::move(containerofcontainers), SkColorSetARGB(100, 125, 125, 125), true);
}


std::unique_ptr<curan::ui::Container> select_registration_mri_ct(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto registervolumes = Button::make("Register Volumes", *appdata.resources);
    registervolumes->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    registervolumes->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("solving registration problem\n");
        DICOMImageType::Pointer ct_input_converted;
        if (auto search = appdata.ct_volumes.find("raw"); search != appdata.ct_volumes.end())
            ct_input_converted = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find CT source dicom image",*appdata.resources));
            return;
        }
        
        DICOMImageType::Pointer mri_input_converted;
        if (auto search = appdata.mri_volumes.find("raw"); search != appdata.mri_volumes.end())
            mri_input_converted = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find MRI source dicom image",*appdata.resources));
            return;
        }

        if (config->stack_page != nullptr) config->stack_page->stack(success_overlay("starting registration",*appdata.resources));

        appdata.pool->submit("",[&, configlocal = config,  fixed_image = ct_input_converted, moving_image = mri_input_converted ](){
            auto [resampled_output,checked_overlap_output] = solve_registration(fixed_image,moving_image,appdata);
            appdata.sync_tasks_with_screen_queue.push(curan::utilities::Job{"emplace volumes in maps",[&, config = configlocal,fixed_image = fixed_image, moving_image = resampled_output, checkered = checked_overlap_output ](){
               DICOMImageType::Pointer fixed_image;
                if (auto search = appdata.ct_volumes.find("raw"); search != appdata.ct_volumes.end())
                    fixed_image = search->second.img;
                else{
                    if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find CT source dicom image",*appdata.resources));
                    return;
                }
                appdata.ct_volumes["source"]=CachedVolume{fixed_image};
                appdata.mri_volumes["source"] = CachedVolume{moving_image};
                appdata.current_volume = "source";
                appdata.miscellaneous_volumes["checkered"] = CachedVolume{checkered};
                if(appdata.modalitytype == ViewType::CT_VIEW){
                    appdata.vol_mas->update_volume(fixed_image);
                } else {
                    appdata.vol_mas->update_volume(moving_image);
                }
                config->stack_page->stack(success_overlay("registration complete!",*appdata.resources));
            }});
        });

    });

    auto validate_checkered = Button::make("Validate Checkered Overlap", *appdata.resources); 
    validate_checkered->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    validate_checkered->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        auto color = button->get_waiting_color();
        if(color == SK_ColorGRAY){ // we need to change into the overlap
           DICOMImageType::Pointer overlapped;
            if (auto search = appdata.miscellaneous_volumes.find("checkered"); search != appdata.miscellaneous_volumes.end())
                overlapped = search->second.img;
            else{
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("registration must be solved before checking overlap",*appdata.resources));
                return;
            }
            button->set_waiting_color(SkColorSetARGB(0xFF, 65, 105, 225));
            appdata.vol_mas->update_volume(overlapped);
        } else {
            if(appdata.modalitytype == ViewType::CT_VIEW){
               DICOMImageType::Pointer fixed_image;
                if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
                    fixed_image = search->second.img;
                else{
                    if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find CT source dicom image",*appdata.resources));
                    return;
                }
                appdata.vol_mas->update_volume(fixed_image);
            } else {
               DICOMImageType::Pointer moving_image;
                if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
                    moving_image = search->second.img;
                else{
                    if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find MRI source dicom image",*appdata.resources));
                    return;
                }
                appdata.vol_mas->update_volume(moving_image);
            }
            button->set_waiting_color(SK_ColorGRAY);
        }

            
    });

    auto switchto = Button::make("Switch Modality", *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("switch representation %s\n",appdata.current_volume.c_str());
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
        std::printf("done switch representation %s\n",appdata.current_volume.c_str());
    });

    auto fixregistration = Button::make("Fix Registration", *appdata.resources);
    fixregistration->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    fixregistration->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(appdata.modalitytype == ViewType::CT_VIEW){ 
           DICOMImageType::Pointer ctinput;
            if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end()){
                ctinput = search->second.img;
            } else {
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("cannot advance without complete registration",*appdata.resources));
                return;
            }
            appdata.vol_mas->update_volume(ctinput);
        } else {
           DICOMImageType::Pointer mriinput;
            if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end()){
                mriinput = search->second.img;
            } else {
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("cannot advance without complete registration",*appdata.resources));
                return;
            }
            appdata.vol_mas->update_volume(mriinput);
        }
        appdata.panel_constructor = select_ac_pc_midline;
        appdata.volume_callback  = ac_pc_midline_point_selection;
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
    });
    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(registervolumes) << std::move(validate_checkered) << std::move(switchto) << std::move(fixregistration);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return container;
};


void ac_pc_midline_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if (strokes.point_in_image_coordinates.cols() > 1)
    {
        config_draw->stack_page->stack(warning_overlay("must select a single point, not a path",*appdata.resources));
        return;
    }
   DICOMImageType::IndexType local_index;
   DICOMImageType::PointType itk_point_in_world_coordinates;
    local_index[0] = strokes.point_in_image_coordinates(0, 0);
    local_index[1] = strokes.point_in_image_coordinates(1, 0);
    local_index[2] = strokes.point_in_image_coordinates(2, 0);
    vol_mas->get_volume()->TransformIndexToPhysicalPoint(local_index, itk_point_in_world_coordinates);
    Eigen::Matrix<double, 3, 1> word_coordinates = Eigen::Matrix<double, 3, 1>::Zero();
    word_coordinates(0, 0) = itk_point_in_world_coordinates[0];
    word_coordinates(1, 0) = itk_point_in_world_coordinates[1];
    word_coordinates(2, 0) = itk_point_in_world_coordinates[2];

    if(appdata.ac_pc_data.ac_specification){
        appdata.ac_pc_data.ac_word_coordinates = word_coordinates;
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.ac_button->set_waiting_color(SkColorSetARGB(0xFF, 0x0F, 0xFF, 0x0F));
        config_draw->stack_page->stack(success_overlay("ac point defined",*appdata.resources));
    }

    if(appdata.ac_pc_data.pc_specification){
        appdata.ac_pc_data.pc_word_coordinates = word_coordinates;
        appdata.ac_pc_data.pc_specification = false;
        appdata.ac_pc_data.pc_button->set_waiting_color(SkColorSetARGB(0xFF, 0x0F, 0xFF, 0x0F));
        config_draw->stack_page->stack(success_overlay("pc point defined",*appdata.resources));
    }
}

std::unique_ptr<curan::ui::Container> select_ac_pc_midline(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto defineac = Button::make("Locate AC", *appdata.resources);
    defineac->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    defineac->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = true;
        appdata.ac_pc_data.pc_specification = false;
    });

    appdata.ac_pc_data.ac_button = defineac.get();

    auto definepc = Button::make("Locate PC", *appdata.resources);
    definepc->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    definepc->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = true;
    });

    appdata.ac_pc_data.pc_button = definepc.get();

    auto resample = Button::make("Resample Volume", *appdata.resources);
    resample->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    resample->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = false;
        std::printf("Resampling volume...\n");
        if(!appdata.ac_pc_data.ac_word_coordinates || !appdata.ac_pc_data.pc_word_coordinates){
            config->stack_page->stack(warning_overlay("must define AC-PC midline before resampling",*appdata.resources));
            return;
        }
        // this vector specified the normal of the coronal plane which corresponds to the xs
        Eigen::Matrix<double, 3, 1> orient_along_ac_pc = *appdata.ac_pc_data.ac_word_coordinates - *appdata.ac_pc_data.pc_word_coordinates;
        if (orient_along_ac_pc.norm() < 1e-7){
            config->stack_page->stack(warning_overlay("AC-PC line is singular, try different points",*appdata.resources));
            return;
        }
        orient_along_ac_pc.normalize(); 
        Eigen::Matrix<double, 3, 1> y_direction = orient_along_ac_pc;

        auto resampler_utility = [&](auto& volumes){
           DICOMImageType::Pointer input;
            if (auto search = volumes.find("source"); search != volumes.end())
                input = search->second.img;
            else{
                return std::make_tuple(false,std::string{"could not find source dicom image"},DICOMImageType::Pointer());
            }

            auto direction = input->GetDirection();
            Eigen::Matrix<double,3,3> eigen_direction;
            Eigen::Matrix<double,3,3> original_eigen_rotation_matrix;
            for(size_t i = 0; i < 3; ++i)
                for(size_t j = 0;  j < 3; ++j){
                    eigen_direction(i,j) = direction(i,j);
                    original_eigen_rotation_matrix(i,j) = direction(i,j);
                }

            Eigen::Matrix<double,3,1> x_direction = -original_eigen_rotation_matrix.col(0);
            Eigen::Matrix<double,3,1> z_direction = x_direction.cross(y_direction);  
            x_direction = y_direction.cross(z_direction);  
            x_direction.normalize();
            z_direction = x_direction.cross(y_direction);  
            z_direction.normalize();
            if (z_direction.norm() < 1e-7)
                return std::make_tuple(false,std::string{"AC-PC line is singular when projected unto the axial plane, try different points"},DICOMImageType::Pointer());

            Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
            eigen_rotation_matrix.col(0) = x_direction;
            eigen_rotation_matrix.col(1) = y_direction;
            eigen_rotation_matrix.col(2) = z_direction;

            //std::cout << "original orientation:\n" << original_eigen_rotation_matrix << std::endl;
            //std::cout << "modified orientation:\n" << eigen_rotation_matrix << std::endl;

            BoundingBox bounding_box_original_image{input};        
            auto output_bounding_box = bounding_box_original_image.centered_bounding_box(eigen_rotation_matrix);
            
            using FilterType = itk::ResampleImageFilter<DICOMImageType,DICOMImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(input);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(DICOMImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
            filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

            itk::Matrix<double> rotation_matrix;
            for (size_t col = 0; col < 3; ++col)
                for (size_t row = 0; row < 3; ++row)
                    rotation_matrix(row, col) = output_bounding_box.orientation(row, col);

            if(output_bounding_box.orientation.determinant() < 0.999 ||  output_bounding_box.orientation.determinant() > 1.001)
                return std::make_tuple(false,std::string{"matrix must be rotation (ortogonal)"},DICOMImageType::Pointer());
                
            filter->SetOutputDirection(rotation_matrix);

            try{
                filter->Update();
               DICOMImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,std::string{"success"},output);
            } catch (...){
                return std::make_tuple(false,std::string{"resampling filter has failed"},DICOMImageType::Pointer());
            }

        };
        auto [resampling_flag_ct,error_description_ct,output_ct] = resampler_utility(appdata.ct_volumes);
        if(!resampling_flag_ct){
            if (config->stack_page != nullptr) {
                config->stack_page->stack(warning_overlay("CT: "+error_description_ct,*appdata.resources));
            }
            return; 
        }
        auto [resampling_flag_mri,error_description_mri,output_mri] = resampler_utility(appdata.mri_volumes);
        if(!resampling_flag_mri){
            if (config->stack_page != nullptr) {
                config->stack_page->stack(warning_overlay("MRI: "+error_description_mri,*appdata.resources));
            }
            return; 
        }

        appdata.ct_volumes["acpc"]=CachedVolume{output_ct};
        appdata.mri_volumes["acpc"]=CachedVolume{output_mri};
        
        if (config->stack_page != nullptr) {
            config->stack_page->stack(success_overlay("pc point defined",*appdata.resources)); 
        }
    });

    auto switch_volume = Button::make("Switch Alignment", *appdata.resources); 
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switch_volume->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = false;
        if(config->stack_page!=nullptr){
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES));
		}
    });

    auto check = Button::make("Trajectory Planning", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(appdata.ac_pc_data.ac_word_coordinates && appdata.ac_pc_data.pc_word_coordinates)
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        else
            config->stack_page->stack(warning_overlay("cannot advance without AC-PC specification",*appdata.resources));

        if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
            appdata.miscellaneous_colored_volumes["ct_source"] = ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.ct_limits,{255.0f,0.0f,0.0f})};
        else{
            config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
            return;
        }

        if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
            appdata.miscellaneous_colored_volumes["mri_source"] = ColoredCachedVolume{DeepColoredCopy<ColorDicomViewer::ImageType,DICOMImageType>(search->second.img,appdata.color_coding.mri_limits,{0.0f,0.0f,255.0f})};
        else{
            config->stack_page->stack(warning_overlay("could not convert to colored representation",*appdata.resources));
            return;
        }
                    
        appdata.panel_constructor = validate_color_coding;
        appdata.volume_callback = {};
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
    });

    auto switchto = Button::make("Switch Modality", *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(defineac) << std::move(definepc) << std::move(resample) << std::move(switch_volume) << std::move(switchto) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return container;
};

std::unique_ptr<curan::ui::Container> validate_color_coding(Application& appdata){
    using namespace curan::ui;
    if(appdata.modalitytype == ViewType::CT_VIEW){
        if (auto search = appdata.miscellaneous_colored_volumes.find("ct_source"); search != appdata.miscellaneous_colored_volumes.end())
            appdata.color_vol_mas.update_volume(search->second.img);
        else
            throw std::runtime_error("failure to find colored images");
    } else {
        if (auto search = appdata.miscellaneous_colored_volumes.find("mri_source"); search != appdata.miscellaneous_colored_volumes.end())
            appdata.color_vol_mas.update_volume(search->second.img);
        else
            throw std::runtime_error("failure to find colored images");
    }

    auto image_display = create_colored_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto switch_modalities = Button::make("Switch Modalities", *appdata.resources);
    switch_modalities->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switch_modalities->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
            if (auto search = appdata.miscellaneous_colored_volumes.find("mri_source"); search != appdata.miscellaneous_colored_volumes.end()){
                appdata.color_vol_mas.update_volume(search->second.img,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
                appdata.modalitytype = ViewType::MRI_VIEW;
            } else{
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.miscellaneous_colored_volumes.find("ct_source"); search != appdata.miscellaneous_colored_volumes.end()){
                appdata.color_vol_mas.update_volume(search->second.img,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
                appdata.modalitytype = ViewType::CT_VIEW;
            } else {   
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
        }
    });

    auto update_intervals = Button::make("Update Intervals", *appdata.resources);
    update_intervals->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    update_intervals->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        config->stack_page->stack(create_slider_range_page(appdata));
    });

    auto confirm_color_coding = Button::make("Confirm Color Coding", *appdata.resources);
    confirm_color_coding->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    confirm_color_coding->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("color coding ct: [%.4f %.4f] mir: [%.4f %.4f]\n",appdata.color_coding.ct_limits[0],appdata.color_coding.ct_limits[1],appdata.color_coding.mri_limits[0],appdata.color_coding.mri_limits[1]);
        if(appdata.ac_pc_data.ac_word_coordinates && appdata.ac_pc_data.pc_word_coordinates){
            appdata.panel_constructor = select_target_and_region_of_entry;
            appdata.volume_callback = select_target_and_region_of_entry_point_selection;
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        }
        else
            config->stack_page->stack(warning_overlay("cannot advance without AC-PC specification",*appdata.resources));
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(switch_modalities) << std::move(update_intervals) << std::move(confirm_color_coding);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return container;
}

void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){

    for(auto& [description,cached] : appdata.ct_volumes){
        std::printf("pointer way before [ct]: %llu\n",(size_t)cached.img.get());
    }
    for(auto& [description,cached] : appdata.mri_volumes){
        std::printf("pointer way before [mri]: %llu\n",(size_t)cached.img.get());
    }

    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if (strokes.point_in_image_coordinates.cols() > 1){
        config_draw->stack_page->stack(warning_overlay("must select a single point, not a path",*appdata.resources));
        return;
    }

   DICOMImageType::IndexType local_index;
   DICOMImageType::PointType itk_point_in_world_coordinates;
    local_index[0] = strokes.point_in_image_coordinates(0, 0);
    local_index[1] = strokes.point_in_image_coordinates(1, 0);
    local_index[2] = strokes.point_in_image_coordinates(2, 0);
    vol_mas->get_volume()->TransformIndexToPhysicalPoint(local_index, itk_point_in_world_coordinates);
    Eigen::Matrix<double, 3, 1> word_coordinates = Eigen::Matrix<double, 3, 1>::Zero();
    word_coordinates(0, 0) = itk_point_in_world_coordinates[0];
    word_coordinates(1, 0) = itk_point_in_world_coordinates[1];
    word_coordinates(2, 0) = itk_point_in_world_coordinates[2];

    if(appdata.trajectory_location.main_diagonal_specification){
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.main_diagonal_word_coordinates = word_coordinates;
        appdata.trajectory_location.main_diagonal_button->set_waiting_color(SkColorSetARGB(0xFF, 0x0F, 0xFF, 0x0F));
        config_draw->stack_page->stack(success_overlay("main diagonal defined",*appdata.resources));
    }

    if(appdata.trajectory_location.target_specification){
        appdata.trajectory_location.target_specification = false;
        appdata.trajectory_location.target_world_coordinates = word_coordinates;
        appdata.trajectory_location.target_button->set_waiting_color(SkColorSetARGB(0xFF, 0x0F, 0xFF, 0x0F));
        config_draw->stack_page->stack(success_overlay("target defined",*appdata.resources));
    }

    if(appdata.trajectory_location.main_diagonal_word_coordinates && appdata.trajectory_location.target_world_coordinates){
        curan::geometry::Piramid geom{curan::geometry::CENTROID_ALIGNED};

        const auto target_world_index = *appdata.trajectory_location.target_world_coordinates;
        const auto main_diagonal_world_index = *appdata.trajectory_location.main_diagonal_word_coordinates;

        Eigen::Matrix<double,3,1> vector_aligned = target_world_index-main_diagonal_world_index;
        double scale = vector_aligned.norm();
        Eigen::Vector3d z_direction = vector_aligned.normalized();
        Eigen::Vector3d x_direction = z_direction;
        x_direction[0] -= 10.0;
        x_direction.normalize();
        Eigen::Vector3d y_direction = (z_direction.cross(x_direction)).normalized();
        x_direction = y_direction.cross(z_direction);
        double base_width = std::tan(0.349066)*scale;

        Eigen::Vector3d b0 = main_diagonal_world_index - base_width*y_direction -  base_width*x_direction;
        Eigen::Vector3d b1 = main_diagonal_world_index + base_width*y_direction -  base_width*x_direction;
        Eigen::Vector3d b2 = main_diagonal_world_index + base_width*y_direction +  base_width*x_direction;
        Eigen::Vector3d b3 = main_diagonal_world_index - base_width*y_direction +  base_width*x_direction;
        Eigen::Vector3d target_local_index = target_world_index;

        geom.geometry.vertices[0][0] = target_local_index[0];
        geom.geometry.vertices[0][1] = target_local_index[1];
        geom.geometry.vertices[0][2] = target_local_index[2];

        geom.geometry.vertices[1][0] = b0[0];
        geom.geometry.vertices[1][1] = b0[1];
        geom.geometry.vertices[1][2] = b0[2];

        geom.geometry.vertices[2][0] = b1[0];
        geom.geometry.vertices[2][1] = b1[1];
        geom.geometry.vertices[2][2] = b1[2];

        geom.geometry.vertices[3][0] = b2[0];
        geom.geometry.vertices[3][1] = b2[1];
        geom.geometry.vertices[3][2] = b2[2];

        geom.geometry.vertices[4][0] = b3[0];
        geom.geometry.vertices[4][1] = b3[1];
        geom.geometry.vertices[4][2] = b3[2];

        for(auto& [description,cached] : appdata.ct_volumes){
            std::printf("pointer before [ct]: %llu\n",(size_t)cached.img.get());
        }
        for(auto& [description,cached] : appdata.mri_volumes){
            std::printf("pointer before [mri]: %llu\n",(size_t)cached.img.get());
        }

        appdata.vol_mas->add_geometry(geom,SK_ColorCYAN);  
    
        for(auto& [description,cached] : appdata.ct_volumes){
            std::printf("pointer after [ct]: %llu\n",(size_t)cached.img.get());
        }
        for(auto& [description,cached] : appdata.mri_volumes){
            std::printf("pointer after [mri]: %llu\n",(size_t)cached.img.get());
        }

        for(size_t i = 0; i < geom.geometry.vertices.size(); ++i){
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][0] = (double)geom.geometry.vertices[i][0];
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][1] = (double)geom.geometry.vertices[i][1];
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][2] = (double)geom.geometry.vertices[i][2];
        }
    }

}

std::unique_ptr<curan::ui::Container> select_target_and_region_of_entry(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto definetarget = Button::make("Define Target", *appdata.resources);
    definetarget->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    definetarget->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = true;        
    });

    appdata.trajectory_location.target_button = definetarget.get();

    auto defineentryregion = Button::make("Define Entry Region", *appdata.resources);
    defineentryregion->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    defineentryregion->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = true;
        appdata.trajectory_location.target_specification = false;
    });

    appdata.trajectory_location.main_diagonal_button = defineentryregion.get();

    auto validadetrajectory = Button::make("Validate Trajectory", *appdata.resources);
    validadetrajectory->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    validadetrajectory->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;

        if(!appdata.trajectory_location.main_diagonal_word_coordinates || !appdata.trajectory_location.target_world_coordinates){
            config->stack_page->stack(warning_overlay("both the target and the entry region must be selected",*appdata.resources));
            return;
        }
        auto convert_to_eigen = [](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
            Eigen::Matrix<double,3,1> converted;
            converted << point[0] , point[1] , point[2];
            return converted;
        };

        auto tip = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[0]);
        auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
        auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);
        auto base2 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[3]);
        auto base3 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[4]);
        auto average = 0.25*base0+0.25*base1+0.25*base2+0.25*base3;

        Eigen::Matrix<double, 3, 1> orient_along_traj = tip-average;
        if (orient_along_traj.norm() < 1e-7){
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("Geometry is compromised",*appdata.resources));
            return;
        }
        orient_along_traj.normalize(); 
        const Eigen::Matrix<double, 3, 1> z_direction = orient_along_traj;

       DICOMImageType::Pointer ct_input;
        if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
            ct_input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

       DICOMImageType::Pointer mri_input;
        if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
            mri_input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        auto resampler = [&](DICOMImageType::Pointer volinput){
            Eigen::Matrix<double,3,1> y_direction = (base1 - base0).normalized();
            Eigen::Matrix<double,3,1> x_direction = y_direction.cross(z_direction);
            x_direction.normalize();
            y_direction = z_direction.cross(x_direction);
            Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
            eigen_rotation_matrix.col(0) = x_direction;
            eigen_rotation_matrix.col(1) = y_direction;
            eigen_rotation_matrix.col(2) = z_direction;

            BoundingBox bounding_box_original_image{volinput};    
            auto output_bounding_box = bounding_box_original_image.centered_bounding_box(eigen_rotation_matrix);
            using FilterType = itk::ResampleImageFilter<DICOMImageType,DICOMImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(volinput);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(DICOMImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
            filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

            itk::Matrix<double> rotation_matrix;
            for (size_t col = 0; col < 3; ++col)
                for (size_t row = 0; row < 3; ++row)
                    rotation_matrix(row, col) = output_bounding_box.orientation(row, col);

            if(output_bounding_box.orientation.determinant() < 0.999 ||  output_bounding_box.orientation.determinant() > 1.001)
                throw std::runtime_error("matrix must be rotation");
            filter->SetOutputDirection(rotation_matrix);

            try{
                filter->Update();
               DICOMImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,std::string{"resampled volume!"},output);
            } catch (...){
                return std::make_tuple(false,std::string{"failed to resample trajectory volume"},DICOMImageType::Pointer());
            }

        };
        auto [flag_ct,error_description_ct,ct_output] = resampler(ct_input);
        if(!flag_ct){
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("CT: "+error_description_ct,*appdata.resources));
            return;
        }
        auto [flag_mri,error_description_mri,mri_output] = resampler(mri_input);
        if(!flag_mri){
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("MRI: "+error_description_mri,*appdata.resources));
            return;
        }

        appdata.ct_volumes["trajectory"] = CachedVolume{ct_output};
        appdata.mri_volumes["trajectory"] = CachedVolume{mri_output};
        if (config->stack_page != nullptr) 
            config->stack_page->stack(success_overlay("resampled volume!",*appdata.resources));
        appdata.panel_constructor = select_entry_point_and_validate_point_selection;
        appdata.volume_callback = std::function<void(Application&,curan::ui::DicomVolumetricMask<std::uint16_t>*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)>{};
        appdata.projected_volume_callback = select_entry_point_and_validate;
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
    });

    auto switch_volume = Button::make("Switch Alignment", *appdata.resources);
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switch_volume->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;
        if(config->stack_page!=nullptr){
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES));
		}
    });

    auto switchto = Button::make("Switch Modality", *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
    });


    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(definetarget) << std::move(defineentryregion) << std::move(validadetrajectory) << std::move(switchto) << std::move(switch_volume);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

void select_entry_point_and_validate(Application& appdata,curan::ui::ColorDicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the columns of the highlighted path must be at least 1");

    if (strokes.point_in_image_coordinates.cols() > 1){
        config_draw->stack_page->stack(warning_overlay("must select a single point, not a path",*appdata.resources));
        return;
    }

    if(appdata.trajectory_location.entry_specification){
        appdata.trajectory_location.entry_specification = false;
        {
            curan::ui::ColorDicomViewer::ImageType::IndexType local_index;
            curan::ui::ColorDicomViewer::ImageType::PointType itk_point_in_world_coordinates;
            local_index[0] = strokes.point_in_image_coordinates(0, 0);
            local_index[1] = strokes.point_in_image_coordinates(1, 0);
            local_index[2] = strokes.point_in_image_coordinates(2, 0);

            vol_mas->get_volume()->TransformIndexToPhysicalPoint(local_index, itk_point_in_world_coordinates);
            Eigen::Matrix<double, 3, 1> entry_word_coordinates = Eigen::Matrix<double, 3, 1>::Zero();
            entry_word_coordinates[0] = itk_point_in_world_coordinates[0];
            entry_word_coordinates[1] = itk_point_in_world_coordinates[1];
            entry_word_coordinates[2] = itk_point_in_world_coordinates[2];
            appdata.trajectory_location.entry_point_word_coordinates = entry_word_coordinates;
        }

       DICOMImageType::Pointer ct_input;
        if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
            ct_input = search->second.img;
        else{
            if (config_draw->stack_page != nullptr) config_draw->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

       DICOMImageType::Pointer mri_input;
        if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
            mri_input = search->second.img;
        else{
            if (config_draw->stack_page != nullptr) config_draw->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }
       DICOMImageType::Pointer input = (appdata.modalitytype == ViewType::CT_VIEW) ? ct_input : mri_input;
        config_draw->stack_page->stack(success_overlay("entry defined",*appdata.resources));

        auto convert_to_eigen = [&](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
            Eigen::Matrix<double,3,1> converted;
            converted[0] = (double)point[0];
            converted[1] = (double)point[1];
            converted[2] = (double)point[2];
            return converted;
        };
        
        Eigen::Matrix<double,3,1> z_direction = (*appdata.trajectory_location.target_world_coordinates-*appdata.trajectory_location.entry_point_word_coordinates).normalized();
        auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
        auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);

        Eigen::Matrix<double,3,1> x_direction = (base1-base0).normalized();
        Eigen::Matrix<double,3,1> y_direction = (z_direction.cross(x_direction)).normalized();
        x_direction = y_direction.cross(z_direction);
        y_direction = z_direction.cross(x_direction);
        x_direction.normalize();
        y_direction.normalize();

        Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
        eigen_rotation_matrix.col(0) = x_direction;
        eigen_rotation_matrix.col(1) = y_direction;
        eigen_rotation_matrix.col(2) = z_direction;
        double det = eigen_rotation_matrix.determinant();

        if(eigen_rotation_matrix.determinant()<0.999 || eigen_rotation_matrix.determinant()>1.0001)
            throw std::runtime_error("determinant is improper");



        BoundingBox bounding_box_original_image{input};    
        auto output_bounding_box = bounding_box_original_image.centered_bounding_box(eigen_rotation_matrix);
        if(output_bounding_box.orientation.determinant() < 0.999 ||  output_bounding_box.orientation.determinant() > 1.001)
            throw std::runtime_error("matrix must be rotation");

        auto resamplervolume = [&](auto inputvol){
            using FilterType = itk::ResampleImageFilter<DICOMImageType,DICOMImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(inputvol);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(DICOMImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
            filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

            itk::Matrix<double> rotation_matrix;
            for (size_t col = 0; col < 3; ++col)
                for (size_t row = 0; row < 3; ++row)
                    rotation_matrix(row, col) = output_bounding_box.orientation(row, col);


            filter->SetOutputDirection(rotation_matrix);

            try{
                filter->Update();
               DICOMImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,output);
            } catch (...){
                return std::make_tuple(false,DICOMImageType::Pointer());
            }
        };

        auto [flag_ct,ct_output] = resamplervolume(ct_input);
        auto [flag_mri,mri_output] = resamplervolume(mri_input);

        if(!flag_ct){
            if (config_draw->stack_page != nullptr) 
                config_draw->stack_page->stack(warning_overlay("CT: failed to resample trajectory volume",*appdata.resources));
            return;
        }

        if(!flag_mri){
            if (config_draw->stack_page != nullptr) 
                config_draw->stack_page->stack(warning_overlay("MRI: failed to resample trajectory volume",*appdata.resources));
            return;
        }
        try{
           DICOMImageType::Pointer image_in_focus;
            if(appdata.modalitytype == ViewType::CT_VIEW)
                image_in_focus = ct_output;
            else
                image_in_focus = mri_output;
            
            appdata.vol_mas->update_volume(image_in_focus,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES);
            appdata.ct_volumes["alongtrajectory"]=CachedVolume{ct_output};
            appdata.mri_volumes["alongtrajectory"]=CachedVolume{mri_output};
            appdata.current_volume = "alongtrajectory";

            auto convert_to_index_coordinates = [&](const Eigen::Matrix<double,3,1>& point){
               DICOMImageType::IndexType local_index;
               DICOMImageType::PointType itk_point_in_world_coordinates;
                itk_point_in_world_coordinates[0] = point[0];
                itk_point_in_world_coordinates[1] = point[1];
                itk_point_in_world_coordinates[2] = point[2];
                appdata.vol_mas->get_volume()->TransformPhysicalPointToIndex(itk_point_in_world_coordinates,local_index);
                auto size = appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize();
                Eigen::Matrix<double,3,1> converted;
                converted[0] = (1.0/size[0])*(double)local_index[0];
                converted[1] = (1.0/size[1])*(double)local_index[1];
                converted[2] = (1.0/size[2])*(double)local_index[2];
                return std::make_tuple(converted,local_index);
            };
            auto target_world_coordinates = *appdata.trajectory_location.target_world_coordinates;
            auto entry_point_word_coordinates = *appdata.trajectory_location.entry_point_word_coordinates;
            const auto [tip_in_local_coords,pixel_tip_coordinates] = convert_to_index_coordinates(target_world_coordinates);
            const auto [entry_in_local_coords,entry_coordinates] = convert_to_index_coordinates(entry_point_word_coordinates);

            Eigen::Matrix<double,3,1> vector_aligned = tip_in_local_coords-entry_in_local_coords;
            Eigen::Matrix<double,4,4> offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();

            // 3 mm to normalized coordinates output_bounding_box.spacing 
            float size_in_mm = 1.5/(0.3333*(output_bounding_box.spacing[0]+output_bounding_box.spacing[1]+output_bounding_box.spacing[2]));
            float radius_in_normalized =  size_in_mm/(0.3333*(output_bounding_box.size[0]+output_bounding_box.size[1]+output_bounding_box.size[2]));
            float trajectory_length = vector_aligned.norm();
            curan::geometry::ClosedCylinder geom{2,100,radius_in_normalized,trajectory_length};
            vector_aligned.normalize();
            Eigen::Matrix<double,3,1> yAxis(0, 0, 1);
            Eigen::Matrix<double,3,1> axis = yAxis.cross(vector_aligned);
            axis.normalize();
            double angle = std::acos(yAxis.transpose()*vector_aligned);
            auto final_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix(); // 
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy(2,3) = -trajectory_length/2.0;  
            geom.transform(offset_base_to_Oxy);
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy.block<3,3>(0,0) = final_rotation;
            geom.transform(offset_base_to_Oxy);
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy.block<3,1>(0,3) = tip_in_local_coords; 
            geom.transform(offset_base_to_Oxy);
            //ok here is the magic, so this transforms the geometry to be correctly aligned inside the cranium in normalized coordiantes right?
            // so what we need to do is to do is to then transform these local coordinates to world coordinates through  
            Eigen::Matrix<double,4,4> local_to_world = Eigen::Matrix<double,4,4>::Identity();
            auto size = image_in_focus->GetLargestPossibleRegion().GetSize();
            auto spacing = image_in_focus->GetSpacing();
            local_to_world(0,0) = spacing[0]*size[0];
            local_to_world(1,1) = spacing[1]*size[1];
            local_to_world(2,2) = spacing[2]*size[2];
            geom.transform(local_to_world);
            local_to_world = Eigen::Matrix<double,4,4>::Identity();
            auto direction = image_in_focus->GetDirection();
            auto origin = image_in_focus->GetOrigin();
            for(size_t i = 0; i < 3; ++i)
                for(size_t j = 0; j < 3; ++j)
                    local_to_world(i,j) = direction(i,j);
            local_to_world(0,3) = origin[0];
            local_to_world(1,3) = origin[1];
            local_to_world(2,3) = origin[2];
            geom.transform(local_to_world);
            appdata.vol_mas->add_geometry(geom,SkColorSetARGB(0xFF, 0x00, 0xFF, 0x00));  
            if (config_draw->stack_page != nullptr) {
                config_draw->stack_page->stack(success_overlay("resampled volume!",*appdata.resources));
            }
        } catch (...){
            if (config_draw->stack_page != nullptr) config_draw->stack_page->stack(warning_overlay("failed to resample trajectory volume",*appdata.resources));
        }
    }
}


std::unique_ptr<curan::ui::Container> select_entry_point_and_validate_point_selection(Application& appdata){
    using namespace curan::ui;

   DICOMImageType::Pointer ct_input;
    if (auto search = appdata.ct_volumes.find("trajectory"); search != appdata.ct_volumes.end())
        ct_input = search->second.img;
    else
        throw std::runtime_error("failure due to missing volume");

   DICOMImageType::Pointer mri_input;
    if (auto search = appdata.mri_volumes.find("trajectory"); search != appdata.mri_volumes.end())
        mri_input = search->second.img;
    else
       throw std::runtime_error("failure due to missing volume");

    if(appdata.modalitytype == ViewType::CT_VIEW)
        appdata.vol_mas->update_volume(ct_input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES);
    else 
        appdata.vol_mas->update_volume(mri_input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES);
    try{                                                                                                                                                                    
        curan::ui::ColorDicomViewer::ImageType::Pointer ct_projected_input = allocate_image_itk_based_phong(appdata,ct_input,appdata.color_coding.ct_limits,{1.0f,0.0f,0.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
        curan::ui::ColorDicomViewer::ImageType::Pointer mri_projected_input = allocate_image_itk_based_phong(appdata,mri_input,appdata.color_coding.mri_limits,{0.0f,0.0f,1.0f},appdata.TransparencyValue,appdata.AlphaFuncValue,appdata.SampleDensityValue);
        appdata.miscellaneous_colored_volumes["ct_projection"] = ColoredCachedVolume{ct_projected_input};
        appdata.miscellaneous_colored_volumes["mri_projection"] = ColoredCachedVolume{mri_projected_input};
        if(appdata.modalitytype == ViewType::CT_VIEW)
            appdata.color_vol_mas.update_volume(ct_projected_input);
        else 
            appdata.color_vol_mas.update_volume(mri_projected_input);
        std::printf("added projection!\n");
    } catch(...){
        std::cout << "failure allocating image" << std::endl;
        throw std::runtime_error("failure");
    }

    auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    auto image_displayx = curan::ui::ColorDicomViewer::make(*appdata.resources, &appdata.color_vol_mas, Direction::Z);
    image_displayx->push_options({"select path","delete path","draw path"});
    image_displayx->add_overlay_processor([&](ColorDicomViewer* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
            switch(selected_option){
                case 0:
                    viewer->change_path_state(ColorPathState::COLORHIGHLIGHTPATH);
                break;
                case 1:
                    viewer->change_path_state(ColorPathState::COLORDELETEPATH);
                    break;
                case 2:
                    viewer->change_path_state(ColorPathState::COLORDRAWPATH);
                    break;
                default:
                    break;
            }
        });
    auto image_displayy = curan::ui::DicomViewer<std::uint16_t>::make(*appdata.resources, appdata.vol_mas, Direction::Z);
    image_displayy->push_options({"coronal view","axial view","saggital view","zoom"});
    image_displayy->add_overlay_processor([&](DicomViewer<std::uint16_t>* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
        switch(selected_option){
            case 0:
                viewer->update_volume(appdata.vol_mas, Direction::X);
            break;
            case 1:
                viewer->update_volume(appdata.vol_mas, Direction::Y);
            break;
            case 2:
                viewer->update_volume(appdata.vol_mas, Direction::Z);
            break;
            case 3:
                viewer->change_zoom();
            break;
            default:
            break;
        }
    });
    *slidercontainer << std::move(image_displayx) << std::move(image_displayy);

    auto defineentry = Button::make("Select Entry Point", *appdata.resources);
    defineentry->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    defineentry->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = true;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;    
    });

    auto update_projection_params = Button::make("Update Projection", *appdata.resources);
    update_projection_params->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    update_projection_params->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        config->stack_page->stack(create_projection_page(appdata));
    });

    auto check = Button::make("Check", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;
        appdata.panel_constructor = select_roi_for_surgery;
        appdata.volume_callback = select_roi_for_surgery_point_selection;
        if(appdata.ac_pc_data.ac_word_coordinates && appdata.ac_pc_data.pc_word_coordinates)
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        else
            config->stack_page->stack(warning_overlay("cannot advance without AC-PC specification",*appdata.resources));
    });

    auto switchto = Button::make("Switch Modality", *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
           DICOMImageType::Pointer input;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                input = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }

            curan::ui::ColorDicomViewer::ImageType::Pointer proj_input;
            if (auto search = appdata.miscellaneous_colored_volumes.find("mri_projection"); search != appdata.miscellaneous_colored_volumes.end())
                proj_input = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            appdata.color_vol_mas.update_volume(proj_input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;

        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer input;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                input = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }

            curan::ui::ColorDicomViewer::ImageType::Pointer proj_input;
            if (auto search = appdata.miscellaneous_colored_volumes.find("ct_projection"); search != appdata.miscellaneous_colored_volumes.end())
                proj_input = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            appdata.color_vol_mas.update_volume(proj_input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(defineentry) << std::move(switchto) << std::move(update_projection_params) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(slidercontainer);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
}

void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::DicomVolumetricMask<std::uint16_t> *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if(appdata.roi.is_selecting_path){
        appdata.roi.is_selecting_path = false;
        appdata.roi.paths.push_back(strokes);
        config_draw->stack_page->stack(success_overlay("added path",*appdata.resources));
    }
}

std::unique_ptr<curan::ui::Container> select_roi_for_surgery(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto addpath = Button::make("Append Path", *appdata.resources);
    addpath->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    addpath->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.roi.is_selecting_path = true;
    });

    auto removelast = Button::make("Remove Last Path", *appdata.resources);
    removelast->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    removelast->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        auto copy = appdata.roi.paths;
        copy.clear();
        for(size_t i = 0; i <= appdata.roi.paths.size(); ++i)
            copy.push_back(appdata.roi.paths[i]);
        appdata.roi.paths = copy;
    });

    auto removeall = Button::make("Remove All Paths", *appdata.resources);
    removeall->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    removeall->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.roi.paths.clear();
    });
    
    auto addroi = Button::make("Add ROI", *appdata.resources);
    addroi->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    addroi->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        Eigen::Matrix<double,3,Eigen::Dynamic> min_and_max_both_paths = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,appdata.roi.paths.size()*2);
        size_t col = 0;
        for(auto path : appdata.roi.paths){
            Eigen::Matrix<double,3,1> min_coeff_all = path.point_in_image_coordinates.rowwise().minCoeff();
            Eigen::Matrix<double,3,1> max_coeff_all = path.point_in_image_coordinates.rowwise().maxCoeff();
            min_and_max_both_paths.col(col) = min_coeff_all;
            min_and_max_both_paths.col(col+1) = max_coeff_all;
            col+=2;
        }
        Eigen::Matrix<double,3,1> min_coeff_all = min_and_max_both_paths.rowwise().minCoeff();
        Eigen::Matrix<double,3,1> max_coeff_all = min_and_max_both_paths.rowwise().maxCoeff();

        min_coeff_all[0] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[0];
        min_coeff_all[1] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[1];
        min_coeff_all[2] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[2];
    
        max_coeff_all[0] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[0];
        max_coeff_all[1] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[1];
        max_coeff_all[2] /= appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[2];

        for(auto& coef : min_coeff_all)
            coef = std::max(coef,0.0);
        for(auto& coef : max_coeff_all)
            coef = std::max(coef,0.0);

        for(auto& coef : min_coeff_all)
            coef = std::min(coef,1.0);
        for(auto& coef : max_coeff_all)
            coef = std::min(coef,1.0);

        Eigen::Matrix<double,3,1> length = max_coeff_all - min_coeff_all;
        Eigen::Matrix<double,3,1> origin = min_coeff_all;

        curan::geometry::Cube geom{1,1,1};

        Eigen::Matrix<double,4,4> rotation_and_scalling_matrix = Eigen::Matrix<double,4,4>::Identity();
        rotation_and_scalling_matrix(0,0) = 0.5;
        rotation_and_scalling_matrix(1,1) = 0.5;
        rotation_and_scalling_matrix(2,2) = 0.5;
        rotation_and_scalling_matrix.block<3,1>(0,3) = Eigen::Matrix<double,3,1>::Ones()*0.5;
        
        // first we rotate the cube from -1 to +1 into the coordinates from 0 to +1
        geom.transform(rotation_and_scalling_matrix);

        rotation_and_scalling_matrix = Eigen::Matrix<double,4,4>::Identity();
        rotation_and_scalling_matrix(0,0) = length[0];
        rotation_and_scalling_matrix(1,1) = length[1];
        rotation_and_scalling_matrix(2,2) = length[2];
        rotation_and_scalling_matrix.block<3,1>(0,3) = origin;

        // now that the cube is between 0 a +1 we can scale it and offset it to be in the coordinates supplied by the user
        geom.transform(rotation_and_scalling_matrix);

        appdata.vol_mas->add_geometry(geom,SkColorSetARGB(0xFF, 0x00, 0x00, 0xFF));
        appdata.roi.paths.clear();
    });

    auto switch_volume = Button::make("Switch Alignment", *appdata.resources);
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switch_volume->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = false;
        if(config->stack_page!=nullptr){
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES));
		}
    });

    auto switchto = Button::make("Switch Modality", *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
           DICOMImageType::Pointer input;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                input = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
           DICOMImageType::Pointer input;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                input = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask<std::uint16_t>::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
        std::printf("done switch representation %s\n",appdata.current_volume.c_str());
    });

    auto check = Button::make("Store Trajectory Data", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));

    auto roi_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *roi_container << std::move(addpath) << std::move(removelast) << std::move(removeall) << std::move(addroi);

    roi_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(switch_volume) << std::move(switchto) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto views_and_roi_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *views_and_roi_container << std::move(roi_container) << std::move(image_display);
    views_and_roi_container->set_divisions({0.0, 0.1, 1.0});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(views_and_roi_container);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

std::unique_ptr<curan::ui::Container> Application::main_page(){
    using namespace curan::ui;
    panel_constructor = select_registration_mri_ct;
    auto container_with_widgets = panel_constructor(*this);
    std::unique_ptr<MiniPage> minipage = MiniPage::make(std::move(container_with_widgets), SK_ColorBLACK);
    tradable_page = minipage.get();
    auto minimage_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *minimage_container << std::move(minipage);
    vol_mas->add_pressedhighlighted_call([this](DicomVolumetricMask<std::uint16_t> *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
        std::cout << "pressed was called!\n";
        if(volume_callback)
            volume_callback(*this,vol_mas,config_draw,strokes);
    });
    color_vol_mas.add_pressedhighlighted_call([this](ColorDicomVolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
        if(projected_volume_callback)
            projected_volume_callback(*this,vol_mas,config_draw,strokes);
    });
    return std::move(minimage_container);
}

#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Loader.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>


using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

std::optional<DICOMImageType::Pointer> get_volume(std::string path, std::string identifier)
{
	using ReaderType = itk::ImageSeriesReader<DICOMImageType>;
	auto reader = ReaderType::New();

	using ImageIOType = itk::GDCMImageIO;
	auto dicomIO = ImageIOType::New();

	reader->SetImageIO(dicomIO);

	using NamesGeneratorType = itk::GDCMSeriesFileNames;
	auto nameGenerator = NamesGeneratorType::New();

	nameGenerator->SetUseSeriesDetails(true);
	nameGenerator->AddSeriesRestriction("0008|0021");

	

	using SeriesIdContainer = std::vector<std::string>;


	bool found = false;
    
    SeriesIdContainer targetFileNames;

    for (const auto& entry : std::filesystem::recursive_directory_iterator(path)) {
        if (entry.is_directory()) {
            nameGenerator->SetDirectory(entry.path().string());
            const std::vector<std::string> &seriesUIDs = nameGenerator->GetSeriesUIDs();
            std::string seriesIdentifier;
            for (const auto& uid : seriesUIDs) {
                std::cout << "uid: " << uid << std::endl;
                if (uid == identifier) {
                    // 2. If found, grab the file names from this specific folder
                    targetFileNames = nameGenerator->GetFileNames(uid);
                    seriesIdentifier = uid;
                    found = true;
                    break; 
                }
            }
        }
    }

    if(!found)
        return std::nullopt;

	reader->SetFileNames(targetFileNames);

	using OrienterType = itk::OrientImageFilter<DICOMImageType, DICOMImageType>;
	auto orienter = OrienterType::New();
	orienter->UseImageDirectionOn(); // Use direction cosines from DICOM

	orienter->SetDesiredCoordinateOrientation(
		itk::SpatialOrientation::ITK_COORDINATE_ORIENTATION_RAS);
	orienter->SetInput(reader->GetOutput());
    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
	auto rescale = RescaleType::New();
	rescale->SetInput(orienter->GetOutput()); // Use oriented image
	rescale->SetOutputMinimum(0);
	rescale->SetOutputMaximum(itk::NumericTraits<std::uint16_t>::max());

	using FilterType = itk::CastImageFilter<DICOMImageType, DICOMImageType>;
	auto filter = FilterType::New();
	filter->SetInput(rescale->GetOutput());

	try
	{
		filter->Update();
		
		// Print orientation information for debugging
		auto orientedImage = orienter->GetOutput();
		//std::cout << "Image Direction: " << orientedImage->GetDirection() << std::endl;
		//std::cout << "Image Origin: " << orientedImage->GetOrigin() << std::endl;
		//std::cout << "Image Spacing: " << orientedImage->GetSpacing() << std::endl;
	}
	catch (const itk::ExceptionObject &ex)
	{
		std::cout << ex << std::endl;
		return std::nullopt;
	}

	return filter->GetOutput();
}

template<typename Image>
typename Image::Pointer missalign(typename Image::Pointer image, std::array<double,3> in_translation, double angle_offset){
    using TransformType = itk::VersorRigid3DTransform<double>;
    auto groundTruthTransform = TransformType::New();

    // Set rotation angle
    TransformType::VersorType rotation;
    {
        itk::Vector<double,3> axis;
        axis[0] = 0.0;
        axis[1] = 1.0;  // rotate around Y-axis
        axis[2] = 0.0;

        axis.Normalize();

        double angleInRadians = itk::Math::pi / angle_offset; // 15 degrees
        rotation.Set(axis, angleInRadians);
    }
    groundTruthTransform->SetRotation(rotation);

    // Set translation
    TransformType::OutputVectorType translation;
    translation[0] = in_translation[0];  // mm
    translation[1] = in_translation[1];  // mm
    translation[2] = in_translation[2];  // mm
    groundTruthTransform->SetTranslation(translation);

    using ResampleFilterType = itk::ResampleImageFilter<Image,Image>;
    auto resampler = ResampleFilterType::New();

    resampler->SetInput(image); // original moving image
    resampler->SetTransform(groundTruthTransform);

    resampler->SetSize(image->GetLargestPossibleRegion().GetSize());
    resampler->SetOutputOrigin(image->GetOrigin());
    resampler->SetOutputSpacing(image->GetSpacing());
    resampler->SetOutputDirection(image->GetDirection());
    resampler->SetDefaultPixelValue(0);
    resampler->Update();

    typename Image::Pointer artificiallyMisalignedImage = resampler->GetOutput();
    return artificiallyMisalignedImage;
}


int main(int argc, char* argv[]) {
    try{
        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();;
        DisplayParams param{ std::move(context),2000,1000};
        param.windowName = "Curan:Santa Maria Demo";
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

        std::printf("\nReading fixed input volume...\n");
        auto fixed_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/DICOM","1.3.46.670589.61.128.1.20250525133838413000200013081989901.2011512512"); // 

        std::printf("\nReading moving input volume...\n");
        auto moving_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/DICOM","1.3.46.670589.11.34212.5.0.12080.20241015103841891.3011480480"); // 
        
        if(!fixed_volume || ! moving_volume)
        {
            if(!fixed_volume) 
                std::printf("failed to read fixed volume\n");
            if(!moving_volume) 
                std::printf("failed to read moving volume\n");
            return 1;
        }


        std::printf("found all moving and fixed volumes\n");

        auto castfilter = itk::CastImageFilter<DICOMImageType,itk::Image<double,3>>::New();
        castfilter->SetInput(*fixed_volume);

        auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
        rescale->SetInput(castfilter->GetOutput());
        rescale->SetOutputMinimum(0);
        rescale->SetOutputMaximum(255.0);

        try{
            rescale->Update();
        }catch (...){
            std::cout << "improperly consumed the input volume";
            return 1;
        }

        DicomVolumetricMask vol{*fixed_volume};
        Application appdata{resources,&vol};
        appdata.ct_volumes["raw"] = CachedVolume{*fixed_volume};
        appdata.mri_volumes["raw"] = CachedVolume{*moving_volume};
        Page page{appdata.main_page(),SK_ColorBLACK};

        page.update_page(viewer.get());

        ConfigDraw config{&page};

        while (!glfwWindowShouldClose(viewer->window)) {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas* canvas = pointer_to_surface->getCanvas();
            if (viewer->was_updated()) {
                page.update_page(viewer.get());
                viewer->update_processed();
            }
            page.draw(canvas);
            auto signals = viewer->process_pending_signals();
            for(auto && sig : signals)
                page.propagate_signal(sig, &config);
            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto async_job = appdata.sync_tasks_with_screen_queue.try_pop();
            if(async_job)
                (*async_job)();
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }


        auto geometries = appdata.vol_mas->geometries();

        std::vector<DICOMImageType::Pointer> internals;
        auto size = appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize();
        internals.reserve(geometries.size());

        auto convert = [&](gte::Vector3<curan::geometry::Piramid::Rational> vertex){
            DICOMImageType::IndexType itk_ind;
            itk_ind[0] = size[0]*(double)vertex[0];
            itk_ind[1] = size[1]*(double)vertex[1];
            itk_ind[2] = size[2]*(double)vertex[2];
            DICOMImageType::PointType point;
            appdata.vol_mas->get_volume()->TransformIndexToPhysicalPoint(itk_ind,point);
            Eigen::Vector3d data;
            data[0] = point[0];
            data[1] = point[1];
            data[2] = point[2];
            return data;
        };

        for (const auto &[key,geomdata] : geometries){
            const auto &[geom,color] = geomdata;
            if(geom.geometry.vertices.size()==8){
                DICOMImageType::Pointer geometry_as_image = DICOMImageType::New();

                auto origin_index = convert(geom.geometry.vertices[0]);
                auto x_dir_index = convert(geom.geometry.vertices[1]);
                auto y_dir_index = convert(geom.geometry.vertices[2]);
                auto z_dir_index = convert(geom.geometry.vertices[4]);

                DICOMImageType::SizeType size;
                size[0] = 10;  // size along X
                size[1] = 10;  // size along Y 
                size[2] = 10;   // size along Z
            
                DICOMImageType::SpacingType spacing;
                spacing[0] = (x_dir_index - origin_index).norm()/(double)size[0]; // mm along X
                spacing[1] = (y_dir_index - origin_index).norm()/(double)size[1]; // mm along X
                spacing[2] = (z_dir_index - origin_index).norm()/(double)size[2]; // mm along X
            
                DICOMImageType::PointType origin;
                origin[0] = origin_index[0];
                origin[1] = origin_index[1];
                origin[2] = origin_index[2];

                Eigen::Matrix<double,3,3> eigen_rotation_matrix;
                eigen_rotation_matrix.col(0) = (x_dir_index - origin_index).normalized();
                eigen_rotation_matrix.col(1) = (y_dir_index - origin_index).normalized();
                eigen_rotation_matrix.col(2) = (z_dir_index - origin_index).normalized();

                auto direction = geometry_as_image->GetDirection();
                for(size_t i = 0; i < 3; ++i)
                    for(size_t j = 0; j < 3; ++j)
                        direction(i,j) = eigen_rotation_matrix(i,j);

                DICOMImageType::RegionType region;
                region.SetSize(size);
            
                geometry_as_image->SetRegions(region);
                geometry_as_image->SetSpacing(spacing);
                geometry_as_image->SetOrigin(origin);
                geometry_as_image->SetDirection(direction);
                geometry_as_image->Allocate();
                geometry_as_image->FillBuffer(0);

                internals.push_back(geometry_as_image);
            }
        }


        auto evaluate_if_pixel_inside_mask = [&](itk::Image<double,3>::IndexType ind,itk::Image<double,3>::Pointer ptr)
        {
            bool is_inside = internals.size() > 0 ? false : true ;
            for (const auto &boundary : internals){
                itk::Image<double,3>::PointType world;
                itk::Image<double,3>::IndexType local_ind;
                ptr->TransformIndexToPhysicalPoint(ind,world);
                boundary->TransformPhysicalPointToIndex(world,local_ind);
                auto size = boundary->GetLargestPossibleRegion().GetSize();
                if ((local_ind[0] >= 0 && local_ind[0] < size[0]) && (local_ind[1] >= 0 && local_ind[1] < size[1]) && (local_ind[2] >= 0 && local_ind[2] < size[2]))
                    is_inside = true;
            }
            return is_inside;
        };

        if(!appdata.trajectory_location.entry_point_word_coordinates || !appdata.trajectory_location.target_world_coordinates){
            std::cout << "Terminating due to unspecified target and entry point" << std::endl;
            return 1;
        }

        Eigen::Matrix<double,3,3> desired_orientation;
        Eigen::Vector3d z_dir = (*appdata.trajectory_location.target_world_coordinates-*appdata.trajectory_location.entry_point_word_coordinates).normalized();
        Eigen::Vector3d x_dir = z_dir;
        x_dir[0] -= 10.0;
        x_dir.normalize();
        Eigen::Vector3d y_dir = z_dir.cross(x_dir);
        x_dir = y_dir.cross(z_dir);
        desired_orientation.col(0) = x_dir;
        desired_orientation.col(1) = y_dir;
        desired_orientation.col(2) = z_dir;

        //std::cout << "desired_orientation:\n" << desired_orientation << std::endl;
        
        auto date = curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::now());
        curan::utilities::TrajectorySpecificationData specification{date,
            *appdata.trajectory_location.target_world_coordinates,
            *appdata.trajectory_location.entry_point_word_coordinates,
            desired_orientation,
            CURAN_COPIED_RESOURCE_PATH"/original_volume.mha",
            CURAN_COPIED_RESOURCE_PATH"/masked_volume.mha"
            };

        auto [masked_output_image,mask_to_use] = DeepCopyWithInclusionPolicy<itk::Image<double,3>>(evaluate_if_pixel_inside_mask,rescale->GetOutput());

        {
            auto writer = itk::ImageFileWriter<itk::Image<double,3>>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/original_volume.mha");
            writer->SetInput(rescale->GetOutput());
            writer->Update();
        }

        {
            auto writer = itk::ImageFileWriter<itk::Image<double,3>>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/masked_volume.mha");
            writer->SetInput(masked_output_image);
            writer->Update();
        }

        {
            auto writer = itk::ImageFileWriter<itk::Image<unsigned char,3>>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/mask.mha");
            writer->SetInput(mask_to_use);
            writer->Update();
        }

        // write prettified JSON to another file
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
        o << specification;
        //std::cout << specification << std::endl;
        
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cout << "Exception thrown:" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
    return 0;
}
