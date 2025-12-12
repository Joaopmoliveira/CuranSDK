#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/DicomDisplay.h"
#include "userinterface/widgets/MiniPage.h"
#include "userinterface/widgets/Page.h"
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

#include <Eigen/Dense>
#include "itkCheckerBoardImageFilter.h"

using DicomPixelType = double;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
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
    ImageType::Pointer img;
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
    std::map<std::string,CachedVolume> ct_volumes;
    std::map<std::string,CachedVolume> mri_volumes;
    std::string current_volume = "raw";
    Eigen::Matrix<double,4,4> registration_fixed_to_moving = Eigen::Matrix<double,4,4>::Identity();
    curan::ui::DicomVolumetricMask* vol_mas = nullptr;
    curan::ui::IconResources* resources = nullptr;
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(2);
    std::function<std::unique_ptr<curan::ui::Container>(Application&)> panel_constructor;
    std::function<void(Application&,curan::ui::DicomVolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> volume_callback;
    curan::ui::DicomVolumetricMask projected_vol_mas{nullptr};
    std::function<void(Application&,curan::ui::DicomVolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> projected_volume_callback;
    RegionOfInterest roi;
    curan::utilities::SafeQueue<curan::utilities::Job> sync_tasks_with_screen_queue;

    //we need to read these when doing the registration pipeline. The point is that the optimizer runs, queries the current location of the slices in the fixed volume
    //computes the intersection with the reoriented image, and then updates the overlays
    std::vector<curan::ui::DicomViewer*> viewers;

    Application(curan::ui::IconResources & in_resources,curan::ui::DicomVolumetricMask* in_vol_mas): resources{&in_resources},vol_mas{in_vol_mas}{}

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
std::unique_ptr<curan::ui::Container> select_registration_mri_ct(Application& appdata);

void ac_pc_midline_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_ac_pc_midline(Application& appdata);
void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_target_and_region_of_entry(Application& appdata);
void select_entry_point_and_validate(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_entry_point_and_validate_point_selection(Application& appdata);
void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_roi_for_surgery(Application& appdata);

template <typename TRegistration>
class RegistrationInterfaceCommand : public itk::Command {
public:
    using Self = RegistrationInterfaceCommand;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

    double m_LastMetricValue;

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
        double currentValue = optimizer->GetValue();
        // Only print out when the Metric value changes
        if (itk::Math::abs(m_LastMetricValue - currentValue) > 1e-7)
        {
            std::cout << optimizer->GetCurrentIteration() << "   ";
            std::cout << currentValue << "   ";
            std::cout << optimizer->GetCurrentPosition() << std::endl;
            m_LastMetricValue = currentValue;
        }
    }

    Application* ptr = nullptr;
    DICOMImageType::Pointer f_fixed;
    DICOMImageType::Pointer f_moving;

    void set(DICOMImageType::Pointer fixed,DICOMImageType::Pointer moving,Application& appdata){
        f_fixed = fixed;
        f_moving = DeepCopy<DICOMImageType>(moving);
        ptr = &appdata;
    };

    ImageType::Pointer resampler(const DICOMImageType::Pointer volume,const Eigen::Matrix<double, 3, 1> &target, const Eigen::Matrix<double, 3, 1> &needle_tip,const Eigen::Matrix<double, 3, 3> &R_ImageToWorld, double &image_size, double &image_spacing)
    {
        using FilterType = itk::ResampleImageFilter<DICOMImageType, DICOMImageType>;
        auto filter = FilterType::New();

        using InterpolatorType = itk::LinearInterpolateImageFunction<DICOMImageType, double>;
        // using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
        auto interpolator = InterpolatorType::New();
        filter->SetInterpolator(interpolator);
        filter->SetDefaultPixelValue(0);

        filter->SetInput(volume);

        itk::Vector<double, 3> spacing;
        spacing[0] = image_spacing;
        spacing[1] = image_spacing;
        spacing[2] = image_spacing;
        filter->SetOutputSpacing(spacing);

        itk::Size<3> size;
        size[0] = image_size;
        size[1] = image_size;
        size[2] = 1;
        filter->SetSize(size);

        Eigen::Vector4d origin_corner_image_frame = Eigen::Vector4d::Ones();
        origin_corner_image_frame[0] = -image_spacing*image_size*0.5;
        origin_corner_image_frame[1] = -image_spacing*image_size*0.5;
        origin_corner_image_frame[2] = (needle_tip-target).transpose()*R_ImageToWorld.col(2);

        Eigen::Matrix<double,4,4> image_frame_transform = Eigen::Matrix<double,4,4>::Identity();
        image_frame_transform.block<3,3>(0,0) = R_ImageToWorld;
        image_frame_transform(0,3) = target[0];
        image_frame_transform(1,3) = target[1];
        image_frame_transform(2,3) = target[2];

        Eigen::Vector4d origin_corner_world_frame = image_frame_transform*origin_corner_image_frame;

        itk::Point<double, 3> origin;
        origin[0] = origin_corner_world_frame[0];
        origin[1] = origin_corner_world_frame[1];
        origin[2] = origin_corner_world_frame[2];
        filter->SetOutputOrigin(origin);

        itk::Matrix<double> image_direction;
        for (size_t row = 0; row < 3; ++row)
            for (size_t col = 0; col < 3; ++col)
                image_direction(col, row) = R_ImageToWorld(col, row);

        filter->SetOutputDirection(image_direction);

        using TransformType = itk::IdentityTransform<double, 3>;
        auto transform = TransformType::New();
        filter->SetTransform(transform);

        try
        {
            filter->Update();
        }
        catch (const itk::ExceptionObject &e)
        {
            std::string result = "Failure to update the filter" + std::string{e.what()};
            std::cout << result;
        }

        return filter->GetOutput();
    }

};

std::tuple<ImageType::Pointer,ImageType::Pointer> solve_registration(DICOMImageType::Pointer fixed_image,DICOMImageType::Pointer moving_image, Application& appdata) {
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

    optimizer->SetLearningRate(8.0);
    optimizer->SetMinimumStepLength(0.0001);
    optimizer->SetReturnBestParametersAndValue(true);
    optimizer->SetRelaxationFactor(0.5);

    typename RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
    shrinkFactorsPerLevel.SetSize(Dimension);
    typename RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
    smoothingSigmasPerLevel.SetSize(Dimension);
    std::array<size_t,Dimension> piramid_sizes{4,2,1};
    std::array<size_t,Dimension> bluering_sizes{3,1,0};
    for (size_t i = 0; i < Dimension; ++i) {
        shrinkFactorsPerLevel[i] = piramid_sizes[i];
        smoothingSigmasPerLevel[i] = bluering_sizes[i];
    }

    registration->SetNumberOfLevels(Dimension);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
    registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

    typename RegistrationType::MetricSamplingStrategyEnum samplingStrategy = RegistrationType::MetricSamplingStrategyEnum::RANDOM;
    registration->MetricSamplingReinitializeSeed(1234);
    registration->SetMetricSamplingStrategy(samplingStrategy);
    registration->SetMetricSamplingPercentage(0.2);


    registration->InPlaceOn();

    using CommandType = RegistrationInterfaceCommand<RegistrationType>;
    auto command = CommandType::New();
    optimizer->AddObserver(itk::IterationEvent(), command);
    command->set(fixed_image,moving_image,appdata);

    try {
        registration->Update();
    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return std::make_tuple(ImageType::Pointer(),ImageType::Pointer());
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
    
    using CastFilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    
    auto caster = CastFilterType::New();
    caster->SetInput(resample->GetOutput());

    try {
        caster->Update();
    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return std::make_tuple(ImageType::Pointer(),ImageType::Pointer());
    }

    
    ImageType::Pointer resampled_output = caster->GetOutput();
    
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
        return std::make_tuple(ImageType::Pointer(),ImageType::Pointer());
    }

    ImageType::Pointer checked_overlap_output = caster->GetOutput();
    return std::make_tuple(resampled_output,checked_overlap_output);
}


ImageType::Pointer allocate_image(Application& appdata,ImageType::Pointer input){
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

    ImageType::Pointer projectionimage = ImageType::New();
  
    ImageType::SizeType size;
    size[0] = 256;  // size along X
    size[1] = 256;  // size along Y 
    size[2] = 1;   // size along Z

    ImageType::SpacingType spacing;
    spacing[0] = (base1 - base0).norm()/(double)size[0]; // mm along X
    spacing[1] = (base3 - base0).norm()/(double)size[1]; // mm along X
    spacing[2] = 1.0; // mm along Z
  
    ImageType::PointType origin;
    origin[0] = base0[0];
    origin[1] = base0[1];
    origin[2] = base0[2];
  
    auto direction = input->GetDirection();
    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            direction(i,j) = eigen_rotation_matrix(i,j);
  
    ImageType::RegionType region;
    region.SetSize(size);
  
    projectionimage->SetRegions(region);
    projectionimage->SetSpacing(spacing);
    projectionimage->SetOrigin(origin);
    projectionimage->SetDirection(direction);
    projectionimage->Allocate();
    projectionimage->FillBuffer(0);

    typedef itk::ImageRegionIteratorWithIndex<ImageType> IteratorType;
    IteratorType it(projectionimage, projectionimage->GetRequestedRegion());

    const Eigen::Matrix<double,3,1> t0 = *appdata.trajectory_location.target_world_coordinates;
    Eigen::Matrix<double,3,1> texcoord;
    
    std::array<std::tuple<ImageType::IndexType,double>,7> neighboors;
    
    auto overall_size = input->GetLargestPossibleRegion().GetSize();

    auto check_pixel_inside = [&](auto tempindex){
        if(tempindex[0] < 0                || tempindex[1] < 0                || tempindex[2] < 0 || 
            tempindex[0] >= overall_size[0] || tempindex[1] >= overall_size[1] || tempindex[2] >= overall_size[2])
             return false;
        else 
            return true;
    };
    ImageType::PointType pos;
    float initial_alpha = 0.0;
    {
        ImageType::IndexType tmpindex;
        pos[0] = t0[0];
        pos[1] = t0[1];
        pos[2] = t0[2];         
        input->TransformPhysicalPointToIndex(pos,tmpindex);
        if(!check_pixel_inside(tmpindex))
            throw std::runtime_error("failure");
        initial_alpha = (1.0/255.0)*input->GetPixel(tmpindex);
    }

    for (it.GoToBegin(); !it.IsAtEnd(); ++it)
    {
        ImageType::PointType pos;
        projectionimage->TransformIndexToPhysicalPoint(it.GetIndex(),pos);

        const Eigen::Matrix<double,3,1> te{pos[0],pos[1],pos[2]};

        const float min_iteratrions = 2.0;
        const float max_iteratrions = 1024.0;

        const float TransparencyValue = 0.01;
        const float AlphaFuncValue = 0.01;
        const float SampleDensityValue = 0.1;
        
        float num_iterations = ceil((te-t0).norm()/SampleDensityValue);
        if (num_iterations<min_iteratrions) num_iterations = min_iteratrions;
        else if (num_iterations>max_iteratrions) num_iterations = max_iteratrions;

        auto mix = [](Eigen::Matrix<double,4,1> l, Eigen::Matrix<double,4,1> r,double mix_ratio){
            Eigen::Matrix<double,3,1> mixed;
            mixed = l.block<3,1>(0,0)*(1.0-mix_ratio)+r.block<3,1>(0,0)*mix_ratio;
            return mixed;
        };


        const Eigen::Matrix<double,3,1> deltaTexCoord = (te-t0)/(num_iterations-1.0);
        texcoord = t0;  
        
        Eigen::Matrix<double,4,1> fragColor{initial_alpha, initial_alpha, initial_alpha, initial_alpha * TransparencyValue};

        auto compute_dist_and_index = [](int neighboorsindex,const Eigen::Matrix<double,3,1>& decimal_coordinates){
            Eigen::Matrix<double,3,1> tmp_to_compute_distance = Eigen::Matrix<double,3,1>::Zero();
            const int offsets[7][3] = {
                {0, 0, 0},
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1},
                {-1, 0, 0}, 
                {0, -1, 0},
                {0, 0, -1}
            };
            ImageType::IndexType indec;
            indec[0] = std::round(decimal_coordinates[0])+offsets[neighboorsindex][0];
            indec[1] = std::round(decimal_coordinates[1])+offsets[neighboorsindex][1];
            indec[2] = std::round(decimal_coordinates[2])+offsets[neighboorsindex][2];
            tmp_to_compute_distance[0] = indec[0];
            tmp_to_compute_distance[1] = indec[1];
            tmp_to_compute_distance[2] = indec[2];
            return std::tuple<ImageType::IndexType,double>(indec,(tmp_to_compute_distance-decimal_coordinates).norm()+1e-5);
        };

        while(num_iterations>0.0){
            ImageType::IndexType current_index_on_input;
            pos[0] = texcoord[0];
            pos[1] = texcoord[1];
            pos[2] = texcoord[2];         
            input->TransformPhysicalPointToIndex(pos,current_index_on_input);
            Eigen::Vector3d eigenized_index_on_input;
            eigenized_index_on_input[0] = current_index_on_input[0];
            eigenized_index_on_input[1] = current_index_on_input[1];
            eigenized_index_on_input[2] = current_index_on_input[2];

            neighboors[0] = compute_dist_and_index(0,eigenized_index_on_input);
            neighboors[1] = compute_dist_and_index(1,eigenized_index_on_input);
            neighboors[2] = compute_dist_and_index(2,eigenized_index_on_input);
            neighboors[3] = compute_dist_and_index(3,eigenized_index_on_input);
            neighboors[4] = compute_dist_and_index(4,eigenized_index_on_input);
            neighboors[5] = compute_dist_and_index(5,eigenized_index_on_input);
            neighboors[6] = compute_dist_and_index(6,eigenized_index_on_input);

            size_t number_of_found_pixels = 0;
            double sum_of_dist = 0.0;
            std::vector<std::tuple<double,double>> neighboors_residue;
            for(size_t in = 0; in < 1 ; ++in){
                const auto& [tmpindex,dist] = neighboors[in];
                if(check_pixel_inside(tmpindex)){
                    ++number_of_found_pixels;
                    sum_of_dist += dist;
                    neighboors_residue.push_back({(1.0/255.0)*input->GetPixel(tmpindex),dist});
                }
            }
            if(number_of_found_pixels==0)
                break;

            double alpha = 0.0;
            for(const auto& [alpha_val,dist] : neighboors_residue )
                alpha += (dist/sum_of_dist)*alpha_val;

            Eigen::Matrix<double,4,1> color{alpha, alpha, alpha, alpha * TransparencyValue};
            float mix_factor = color[3];
            if (mix_factor > AlphaFuncValue){
                fragColor.block<3,1>(0,0) = mix(fragColor, color, mix_factor);
                fragColor[3] += mix_factor;
            }

            if (mix_factor > fragColor[3])
                fragColor = color;

            texcoord += deltaTexCoord;
            --num_iterations;
        }
        it.Set(255.0*fragColor[0]);
  }
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

    BoundingBox(ImageType::Pointer image){
        Eigen::Matrix<double, 3, 1> origin_for_bounding_box{{image->GetOrigin()[0], image->GetOrigin()[1], image->GetOrigin()[2]}};
        ImageType::PointType itk_along_dimension_x;
        ImageType::IndexType index_along_x{{(long long)image->GetLargestPossibleRegion().GetSize()[0], 0, 0}};
        image->TransformIndexToPhysicalPoint(index_along_x, itk_along_dimension_x);
        Eigen::Matrix<double, 3, 1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0], itk_along_dimension_x[1], itk_along_dimension_x[2]}};
        ImageType::PointType itk_along_dimension_y;
        ImageType::IndexType index_along_y{{0, (long long)image->GetLargestPossibleRegion().GetSize()[1], 0}};
        image->TransformIndexToPhysicalPoint(index_along_y, itk_along_dimension_y);
        Eigen::Matrix<double, 3, 1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0], itk_along_dimension_y[1], itk_along_dimension_y[2]}};
        ImageType::PointType itk_along_dimension_z;
        ImageType::IndexType index_along_z{{0, 0, (long long)image->GetLargestPossibleRegion().GetSize()[2]}};
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
    { appdata.type = LayoutType::ONE; appdata.viewers = std::vector<curan::ui::DicomViewer*>{}; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto double_view_layout = Button::make(" ", "layout1x2.png", *appdata.resources);
    double_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    double_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { appdata.type = LayoutType::TWO; appdata.viewers = std::vector<curan::ui::DicomViewer*>{}; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto triple_view_layout = Button::make(" ", "layout1x3.png", *appdata.resources);
    triple_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    triple_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { appdata.type = LayoutType::THREE; appdata.viewers = std::vector<curan::ui::DicomViewer*>{}; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(single_view_layout) << std::move(double_view_layout) << std::move(triple_view_layout);
    viwers_container->set_color(SK_ColorTRANSPARENT);
    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata){
    using namespace curan::ui;
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    auto overlay_lambda = [&](DicomViewer* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
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
                viewer->change_path_selection();
            break;
            default:
            break;
        }
    };
    switch(appdata.type){
        case ONE:
        {
            auto image_display = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::X);
            image_display->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_display->add_overlay_processor(overlay_lambda);
            appdata.viewers.push_back(image_display.get());
            *container << std::move(image_display);
        }
        break;
        case TWO:
        {
            auto image_displayx = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::X);
            image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_displayy->add_overlay_processor(overlay_lambda);
            appdata.viewers.push_back(image_displayx.get());
            appdata.viewers.push_back(image_displayy.get());
            *container << std::move(image_displayx) << std::move(image_displayy);
        }
        break;
        case THREE:
        {
            auto image_displayx = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::X);
            image_displayx->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_displayx->add_overlay_processor(overlay_lambda);
            auto image_displayy = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            image_displayy->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_displayy->add_overlay_processor(overlay_lambda);
            auto image_displayz = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::Z);
            image_displayz->push_options({"coronal view","axial view","saggital view","zoom","select path"});
            image_displayz->add_overlay_processor(overlay_lambda);
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
    using PixelType = unsigned char;
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
    using ImageType = itk::Image<PixelType, 3>;
    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;
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

                ImageType::RegionType inputRegion = itk_pointer->GetBufferedRegion();
                ImageType::SpacingType spacing = itk_pointer->GetSpacing();
                ImageType::SizeType size = inputRegion.GetSize();

                auto copy_size = size;
                size[Direction::Z] = 1;

                ImageType::IndexType start = inputRegion.GetIndex();
                start[Direction::Z] = std::floor(copy_size[Direction::Z] / 2.0);
                ImageType::RegionType desiredRegion;
                desiredRegion.SetSize(size);
                desiredRegion.SetIndex(start);
                extract_filter->SetExtractionRegion(desiredRegion);
                extract_filter->UpdateLargestPossibleRegion();

                ImageType::Pointer pointer_to_block_of_memory = extract_filter->GetOutput();
                ImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
                auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(PixelType), pointer_to_block_of_memory);
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
        {
            ImageType::Pointer ct_input;
            if (auto search = appdata.ct_volumes.find("raw"); search != appdata.ct_volumes.end())
                ct_input = search->second.img;
            else{
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find CT source dicom image",*appdata.resources));
                return;
            }

            using FilterType = itk::CastImageFilter<ImageType,DICOMImageType>;
            auto filter = FilterType::New();
            filter->SetInput(ct_input);

            try {
                filter->Update();
            } catch (const itk::ExceptionObject &err) {
                std::cout << "ExceptionObject caught !" << std::endl;
                std::cout << err << std::endl;
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("failed to convert CT scan",*appdata.resources));
                return;
            }

            ct_input_converted = filter->GetOutput();

        }
        DICOMImageType::Pointer mri_input_converted;
        {
            ImageType::Pointer mri_input;
            if (auto search = appdata.mri_volumes.find("raw"); search != appdata.mri_volumes.end())
                mri_input = search->second.img;
            else{
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find MRI source dicom image",*appdata.resources));
                return;
            }

            using FilterType = itk::CastImageFilter<ImageType,DICOMImageType>;
            auto filter = FilterType::New();
            filter->SetInput(mri_input);
            try {
                filter->Update();
            } catch (const itk::ExceptionObject &err) {
                std::cout << "ExceptionObject caught !" << std::endl;
                std::cout << err << std::endl;
                if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("failed to convert CT scan",*appdata.resources));
                return;
            }
            mri_input_converted = filter->GetOutput();
        }

        if (config->stack_page != nullptr) config->stack_page->stack(success_overlay("starting registration",*appdata.resources));

        appdata.pool->submit("",[&, fixed_image = ct_input_converted, moving_image = mri_input_converted ](){
            auto [resampled_output,checked_overlap_output] = solve_registration(fixed_image,moving_image,appdata);
            appdata.sync_tasks_with_screen_queue.push(curan::utilities::Job{"emplace volumes in maps",[&,fixed_image = fixed_image, moving_image = resampled_output ](){

                ImageType::Pointer fixed_image;
                if (auto search = appdata.ct_volumes.find("raw"); search != appdata.ct_volumes.end())
                    fixed_image = search->second.img;
                else{
                    if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find CT source dicom image",*appdata.resources));
                    return;
                }

                appdata.ct_volumes.emplace("source",fixed_image);
                appdata.mri_volumes.emplace("source",moving_image);
                appdata.current_volume = "source";

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

    auto validate_blending = Button::make("Validate Blending", *appdata.resources);
    validate_blending->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));

    const std::string displaystring = (appdata.modalitytype == ViewType::CT_VIEW) ? "Switch to MRI" : "Switch to CT" ;
    auto switchto = Button::make(displaystring, *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("switch representation %s\n",appdata.current_volume.c_str());
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
            ImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
            ImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
        std::printf("done switch representation %s\n",appdata.current_volume.c_str());
    });

    auto fixregistration = Button::make("Fix Registration", *appdata.resources);
    fixregistration->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    fixregistration->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if (auto search = appdata.mri_volumes.find("source"); search == appdata.mri_volumes.end()){
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("cannot advance without registration",*appdata.resources));
            return;
        }

        if (auto search = appdata.ct_volumes.find("source"); search == appdata.ct_volumes.end()){
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("cannot advance without registration",*appdata.resources));
            return;
        }

        appdata.panel_constructor = select_ac_pc_midline;
        appdata.volume_callback  = ac_pc_midline_point_selection;
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
    });
    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(registervolumes) << std::move(validate_checkered) << std::move(validate_blending) << std::move(switchto) << std::move(fixregistration);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return container;
};


void ac_pc_midline_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if (strokes.point_in_image_coordinates.cols() > 1)
    {
        config_draw->stack_page->stack(warning_overlay("must select a single point, not a path",*appdata.resources));
        return;
    }
    ImageType::IndexType local_index;
    ImageType::PointType itk_point_in_world_coordinates;
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
            ImageType::Pointer input;
            if (auto search = volumes.find("source"); search != volumes.end())
                input = search->second.img;
            else{
                return std::make_tuple(false,std::string{"could not find source dicom image"},ImageType::Pointer());
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
                return std::make_tuple(false,std::string{"AC-PC line is singular when projected unto the axial plane, try different points"},ImageType::Pointer());

            Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
            eigen_rotation_matrix.col(0) = x_direction;
            eigen_rotation_matrix.col(1) = y_direction;
            eigen_rotation_matrix.col(2) = z_direction;

            std::cout << "original orientation:\n" << original_eigen_rotation_matrix << std::endl;
            std::cout << "modified orientation:\n" << eigen_rotation_matrix << std::endl;

            BoundingBox bounding_box_original_image{input};        
            auto output_bounding_box = bounding_box_original_image.centered_bounding_box(eigen_rotation_matrix);
            
            using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(input);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(ImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
            filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

            itk::Matrix<double> rotation_matrix;
            for (size_t col = 0; col < 3; ++col)
                for (size_t row = 0; row < 3; ++row)
                    rotation_matrix(row, col) = output_bounding_box.orientation(row, col);

            if(output_bounding_box.orientation.determinant() < 0.999 ||  output_bounding_box.orientation.determinant() > 1.001)
                return std::make_tuple(false,std::string{"matrix must be rotation (ortogonal)"},ImageType::Pointer());
                
            filter->SetOutputDirection(rotation_matrix);

            try{
                filter->Update();
                ImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,std::string{"success"},output);
            } catch (...){
                return std::make_tuple(false,std::string{"resampling filter has failed"},ImageType::Pointer());
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
        appdata.ct_volumes.emplace("acpc",output_ct);
        appdata.mri_volumes.emplace("acpc",output_mri);
        
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
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES));
		}
    });

    auto check = Button::make("Trajectory Planning", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.panel_constructor = select_target_and_region_of_entry;
        appdata.volume_callback = select_target_and_region_of_entry_point_selection;
        if(appdata.ac_pc_data.ac_word_coordinates && appdata.ac_pc_data.pc_word_coordinates)
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        else
            config->stack_page->stack(warning_overlay("cannot advance without AC-PC specification",*appdata.resources));
    });
    const std::string displaystring = (appdata.modalitytype == ViewType::CT_VIEW) ? "Switch to MRI" : "Switch to CT" ;
    auto switchto = Button::make(displaystring, *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("switch representation %s\n",appdata.current_volume.c_str());
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
            ImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
            ImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
        std::printf("done switch representation %s\n",appdata.current_volume.c_str());
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(defineac) << std::move(definepc) << std::move(resample) << std::move(switch_volume) << std::move(switchto) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return container;
};

void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){

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

    ImageType::IndexType local_index;
    ImageType::PointType itk_point_in_world_coordinates;
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

        ImageType::Pointer ct_input;
        if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
            ct_input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        ImageType::Pointer mri_input;
        if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
            mri_input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        auto resampler = [&](ImageType::Pointer volinput){
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
            using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(volinput);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(ImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
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
                ImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,std::string{"resampled volume!"},output);
            } catch (...){
                return std::make_tuple(false,std::string{"failed to resample trajectory volume"},ImageType::Pointer());
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

        appdata.ct_volumes.emplace("trajectory",ct_output);
        appdata.mri_volumes.emplace("trajectory",mri_output);
        if (config->stack_page != nullptr) 
            config->stack_page->stack(success_overlay("resampled volume!",*appdata.resources));
        appdata.panel_constructor = select_entry_point_and_validate_point_selection;
        appdata.volume_callback = std::function<void(Application&,curan::ui::DicomVolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)>{};
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
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES));
		}
    });
    const std::string displaystring = (appdata.modalitytype == ViewType::CT_VIEW) ? "Switch to MRI" : "Switch to CT" ;
    auto switchto = Button::make(displaystring, *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("switch representation %s\n",appdata.current_volume.c_str());
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
            ImageType::Pointer localinput;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                localinput = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
            ImageType::Pointer localinput;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                localinput = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(localinput,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::CT_VIEW;
        }
        std::printf("done switch representation %s\n",appdata.current_volume.c_str());
    });


    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(definetarget) << std::move(defineentryregion) << std::move(validadetrajectory) << std::move(switchto) << std::move(switch_volume);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

void select_entry_point_and_validate(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
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
            ImageType::IndexType local_index;
            ImageType::PointType itk_point_in_world_coordinates;
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

        ImageType::Pointer ct_input;
        if (auto search = appdata.ct_volumes.find("source"); search != appdata.ct_volumes.end())
            ct_input = search->second.img;
        else{
            if (config_draw->stack_page != nullptr) config_draw->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        ImageType::Pointer mri_input;
        if (auto search = appdata.mri_volumes.find("source"); search != appdata.mri_volumes.end())
            mri_input = search->second.img;
        else{
            if (config_draw->stack_page != nullptr) config_draw->stack_page->stack(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }
        ImageType::Pointer input = (appdata.modalitytype == ViewType::CT_VIEW) ? ct_input : mri_input;
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
            using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
            auto filter = FilterType::New();

            using TransformType = itk::IdentityTransform<double, 3>;
            auto transform = TransformType::New();

            using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
            auto interpolator = InterpolatorType::New();
            filter->SetInterpolator(interpolator);
            filter->SetDefaultPixelValue(0);
            filter->SetTransform(transform);

            filter->SetInput(inputvol);
            filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
            filter->SetOutputSpacing(ImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
            filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

            itk::Matrix<double> rotation_matrix;
            for (size_t col = 0; col < 3; ++col)
                for (size_t row = 0; row < 3; ++row)
                    rotation_matrix(row, col) = output_bounding_box.orientation(row, col);


            filter->SetOutputDirection(rotation_matrix);

            try{
                filter->Update();
                ImageType::Pointer output = filter->GetOutput();
                return std::make_tuple(true,output);
            } catch (...){
                return std::make_tuple(false,ImageType::Pointer());
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
            ImageType::Pointer image_in_focus;
            if(appdata.modalitytype == ViewType::CT_VIEW)
                image_in_focus = ct_output;
            else
                image_in_focus = mri_output;
            
            appdata.vol_mas->update_volume(image_in_focus,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES);
            appdata.ct_volumes.emplace("alongtrajectory",CachedVolume{ct_output});
            appdata.mri_volumes.emplace("alongtrajectory",CachedVolume{mri_output});

            auto convert_to_index_coordinates = [&](const Eigen::Matrix<double,3,1>& point){
                ImageType::IndexType local_index;
                ImageType::PointType itk_point_in_world_coordinates;
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

    ImageType::Pointer ct_input;
    if (auto search = appdata.ct_volumes.find("trajectory"); search != appdata.ct_volumes.end())
        ct_input = search->second.img;
    else
        throw std::runtime_error("failure due to missing volume");

    ImageType::Pointer mri_input;
    if (auto search = appdata.mri_volumes.find("trajectory"); search != appdata.mri_volumes.end())
        mri_input = search->second.img;
    else
       throw std::runtime_error("failure due to missing volume");

    if(appdata.modalitytype == ViewType::CT_VIEW)
        appdata.vol_mas->update_volume(ct_input,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES);
    else 
        appdata.vol_mas->update_volume(mri_input,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES);
    try{
        ImageType::Pointer ct_projected_input = allocate_image(appdata,ct_input);
        ImageType::Pointer mri_projected_input = allocate_image(appdata,mri_input);

        //appdata.ct_volumes.emplace("tmp_proj",CachedVolume{ct_projected_input,false});
        //appdata.mri_volumes.emplace("tmp_proj",CachedVolume{mri_projected_input,false});
        if(appdata.modalitytype == ViewType::CT_VIEW)
            appdata.projected_vol_mas.update_volume(ct_projected_input);
        else 
            appdata.projected_vol_mas.update_volume(mri_projected_input);
        std::printf("added projection!\n");
    } catch(...){
        std::cout << "failure allocating image" << std::endl;
        throw std::runtime_error("failure");
    }

    auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    auto image_displayx = curan::ui::DicomViewer::make(*appdata.resources, &appdata.projected_vol_mas, Direction::Z);
    auto image_displayy = curan::ui::DicomViewer::make(*appdata.resources, appdata.vol_mas, Direction::Z);
    image_displayy->push_options({"coronal view","axial view","saggital view","zoom"});
    image_displayy->add_overlay_processor([&](DicomViewer* viewer, curan::ui::ConfigDraw* config, size_t selected_option){
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

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(defineentry) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(slidercontainer);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
}

void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::DicomVolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
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
			config->stack_page->stack(create_volume_explorer_page(appdata,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES));
		}
    });

    const std::string displaystring = (appdata.modalitytype == ViewType::CT_VIEW) ? "Switch to MRI" : "Switch to CT" ;
    auto switchto = Button::make(displaystring, *appdata.resources);
    switchto->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 80));
    switchto->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        std::printf("switch representation %s\n",appdata.current_volume.c_str());
        // so first I need to check which modality we are currently under
        if(appdata.modalitytype == ViewType::CT_VIEW){ // if we are in ct mode then we want to go to mri
            ImageType::Pointer input;
            if (auto search = appdata.mri_volumes.find(appdata.current_volume); search != appdata.mri_volumes.end())
                input = search->second.img;
            else{
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to mri volume\n");
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
            appdata.modalitytype = ViewType::MRI_VIEW;
        } else { // if we are in mri mode then we want to go to ct
            ImageType::Pointer input;
            if (auto search = appdata.ct_volumes.find(appdata.current_volume); search != appdata.ct_volumes.end())
                input = search->second.img;
            else{   
                std::printf("did not find the volume\n");
                config->stack_page->stack(warning_overlay("Cannot change to MRI view",*appdata.resources));
                return;
            }
            std::printf("updating to ct volume\n");
            appdata.vol_mas->update_volume(input,curan::ui::DicomVolumetricMask::Policy::UPDATE_GEOMETRIES|curan::ui::DicomVolumetricMask::Policy::UPDATE_POINTS);
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
    vol_mas->add_pressedhighlighted_call([this](DicomVolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
        if(volume_callback)
            volume_callback(*this,vol_mas,config_draw,strokes);
    });
    projected_vol_mas.add_pressedhighlighted_call([this](DicomVolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
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


using PixelType = unsigned char;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

std::optional<ImageType::Pointer> get_volume(std::string path, std::string identifier)
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

	nameGenerator->SetDirectory(path);

	using SeriesIdContainer = std::vector<std::string>;

	const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();
    std::string seriesIdentifier;
	auto seriesItr = seriesUID.begin();
	auto seriesEnd = seriesUID.end();
	bool found = false;
    while (seriesItr != seriesEnd)
	{
        if(identifier.compare(std::string(seriesItr->c_str()))==0){
            seriesIdentifier = std::string(seriesItr->c_str());
            found = true;
        }
		//std::cout << seriesItr->c_str() << std::endl;
		++seriesItr;
	}

    if(!found)
        return std::nullopt;

	using FileNamesContainer = std::vector<std::string>;
	FileNamesContainer fileNames;

	fileNames = nameGenerator->GetFileNames(seriesIdentifier);

	reader->SetFileNames(fileNames);

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
	rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

	using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
	auto filter = FilterType::New();
	filter->SetInput(rescale->GetOutput());

	try
	{
		filter->Update();
		
		// Print orientation information for debugging
		auto orientedImage = orienter->GetOutput();
		std::cout << "Image Direction: " << orientedImage->GetDirection() << std::endl;
		std::cout << "Image Origin: " << orientedImage->GetOrigin() << std::endl;
		std::cout << "Image Spacing: " << orientedImage->GetSpacing() << std::endl;
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

int notmain(int argc, char* argv[]) {
    std::printf("\nReading input volume...\n");
    auto fixed_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/ST983524","1.3.46.670589.11.80629.5.0.3932.2021030514141108000.402551251220210305"); // 
    std::printf("\nReading input volume...\n");
    auto moving_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/ST983524","1.3.46.670589.11.80629.5.0.10680.2021030514141216000.403551251220210305"); // 

    if(!fixed_volume || ! moving_volume)
    {
        std::printf("failed to read moving or fixed volume");
        return 1;
    }

    DICOMImageType::Pointer ct_input_converted;
    {
        using FilterType = itk::CastImageFilter<ImageType,DICOMImageType>;
        auto filter = FilterType::New();
        filter->SetInput(*fixed_volume);

        try {
            filter->Update();
        } catch (const itk::ExceptionObject &err) {
            return 1;
        }

        ct_input_converted = filter->GetOutput();

    }
    DICOMImageType::Pointer mri_input_converted;
    {
        using FilterType = itk::CastImageFilter<ImageType,DICOMImageType>;
        auto filter = FilterType::New();
        filter->SetInput(*moving_volume);
        try {
            filter->Update();
        } catch (const itk::ExceptionObject &err) {
            return 1;
        }
        mri_input_converted = filter->GetOutput();
    }
    using namespace curan::ui;
    IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
    Application appdata{resources,nullptr};

    using TransformType = itk::VersorRigid3DTransform<double>;
    auto groundTruthTransform = TransformType::New();

    // Set rotation angle
    TransformType::VersorType rotation;
    {
        itk::Vector<double,3> axis;
        axis[0] = 0.0;
        axis[1] = 1.0;  // rotate around Y-axis
        axis[2] = 0.0;

        double angleInRadians = itk::Math::pi / 12.0; // 15 degrees
        rotation.Set(axis, angleInRadians);
    }
    groundTruthTransform->SetRotation(rotation);

    // Set translation
    TransformType::OutputVectorType translation;
    translation[0] = 5.0;  // mm
    translation[1] = -3.0;
    translation[2] = 8.0;
    groundTruthTransform->SetTranslation(translation);

    using ResampleFilterType = itk::ResampleImageFilter<DICOMImageType, DICOMImageType>;
    auto resampler = ResampleFilterType::New();

    resampler->SetInput(mri_input_converted); // original moving image
    resampler->SetTransform(groundTruthTransform);

    resampler->SetSize(mri_input_converted->GetLargestPossibleRegion().GetSize());
    resampler->SetOutputOrigin(mri_input_converted->GetOrigin());
    resampler->SetOutputSpacing(mri_input_converted->GetSpacing());
    resampler->SetOutputDirection(mri_input_converted->GetDirection());
    resampler->SetDefaultPixelValue(0);
    resampler->Update();

    DICOMImageType::Pointer artificiallyMisalignedImage = resampler->GetOutput();

    auto [resampled_output,checked_overlap_output] = solve_registration(ct_input_converted,artificiallyMisalignedImage,appdata);
    std::printf("finished registration!!!!!!!!\n");

    {
        itk::ImageFileWriter<DICOMImageType>::Pointer warpedImageWriter;
        warpedImageWriter = itk::ImageFileWriter<DICOMImageType>::New();
        warpedImageWriter->SetInput(ct_input_converted);
        warpedImageWriter->SetFileName("fixed.mha");
        try{
            warpedImageWriter->Update();
        } catch (const itk::ExceptionObject & excp){
            std::cerr << excp << std::endl;
            return EXIT_FAILURE;
        }
    }

    {
        itk::ImageFileWriter<DICOMImageType>::Pointer warpedImageWriter;
        warpedImageWriter = itk::ImageFileWriter<DICOMImageType>::New();
        warpedImageWriter->SetInput(artificiallyMisalignedImage);
        warpedImageWriter->SetFileName("moving.mha");
        try{
            warpedImageWriter->Update();
        } catch (const itk::ExceptionObject & excp){
            std::cerr << excp << std::endl;
            return EXIT_FAILURE;
        }
    }

    {
        itk::ImageFileWriter<ImageType>::Pointer warpedImageWriter;
        warpedImageWriter = itk::ImageFileWriter<ImageType>::New();
        warpedImageWriter->SetInput(resampled_output);
        warpedImageWriter->SetFileName("moving_overlayed.mha");
        try{
            warpedImageWriter->Update();
        } catch (const itk::ExceptionObject & excp){
            std::cerr << excp << std::endl;
            return EXIT_FAILURE;
        }
    }

    return 0;
};

int main(int argc, char* argv[]) {
    try{
        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();;
        DisplayParams param{ std::move(context),2000,1000};
        param.windowName = "Curan:Santa Maria Demo";
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

        std::printf("\nReading input volume...\n");
        auto fixed_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/ST983524","1.3.46.670589.11.80629.5.0.3932.2021030514141108000.402551251220210305"); // 
        std::printf("\nReading input volume...\n");
        auto optional_moving_volume =  get_volume("C:/Dev/CuranSDK/resources/dicom_sample/ST983524","1.3.46.670589.11.80629.5.0.10680.2021030514141216000.403551251220210305"); // 
        
        if(!fixed_volume || ! optional_moving_volume)
        {
            std::printf("failed to read moving or fixed volume\n");
            return 1;
        }
        std::array<double,3> spacing{5.0,-3.0,8.0};
        auto moving_volume = missalign<ImageType>(*optional_moving_volume,spacing, 12);

        std::printf("found all moving and fixed volumes\n");

        auto castfilter = itk::CastImageFilter<ImageType,itk::Image<double,3>>::New();
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
        appdata.ct_volumes.emplace("raw",*fixed_volume);
        appdata.mri_volumes.emplace("raw",moving_volume);
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
            if (!signals.empty())
                page.propagate_signal(signals.back(), &config);
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

        std::vector<ImageType::Pointer> internals;
        auto size = appdata.vol_mas->get_volume()->GetLargestPossibleRegion().GetSize();
        internals.reserve(geometries.size());

        auto convert = [&](gte::Vector3<curan::geometry::Piramid::Rational> vertex){
            ImageType::IndexType itk_ind;
            itk_ind[0] = size[0]*(double)vertex[0];
            itk_ind[1] = size[1]*(double)vertex[1];
            itk_ind[2] = size[2]*(double)vertex[2];
            ImageType::PointType point;
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
                ImageType::Pointer geometry_as_image = ImageType::New();

                auto origin_index = convert(geom.geometry.vertices[0]);
                auto x_dir_index = convert(geom.geometry.vertices[1]);
                auto y_dir_index = convert(geom.geometry.vertices[2]);
                auto z_dir_index = convert(geom.geometry.vertices[4]);

                ImageType::SizeType size;
                size[0] = 10;  // size along X
                size[1] = 10;  // size along Y 
                size[2] = 10;   // size along Z
            
                ImageType::SpacingType spacing;
                spacing[0] = (x_dir_index - origin_index).norm()/(double)size[0]; // mm along X
                spacing[1] = (y_dir_index - origin_index).norm()/(double)size[1]; // mm along X
                spacing[2] = (z_dir_index - origin_index).norm()/(double)size[2]; // mm along X
            
                ImageType::PointType origin;
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

                ImageType::RegionType region;
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

        std::cout << "desired_orientation:\n" << desired_orientation << std::endl;
        
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