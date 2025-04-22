#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/SliderPanel.h"
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

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

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

struct CachedVolume{
    ImageType::Pointer img;
};

enum LayoutType{
    ONE,
    TWO,
    THREE
};

struct Application;

struct Application{
    curan::ui::MiniPage* tradable_page = nullptr;
    ACPCData ac_pc_data;
    TrajectoryConeData trajectory_location;
    LayoutType type = LayoutType::THREE;
    std::map<std::string,CachedVolume> volumes;
    size_t volume_index = 0;
    curan::ui::VolumetricMask* vol_mas = nullptr;
    curan::ui::IconResources* resources = nullptr;
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(2);
    std::function<std::unique_ptr<curan::ui::Container>(Application&)> panel_constructor;
    std::function<void(Application&,curan::ui::VolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> volume_callback;
    curan::ui::VolumetricMask projected_vol_mas{nullptr};
    std::function<void(Application&,curan::ui::VolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)> projected_volume_callback;

    Application(curan::ui::IconResources & in_resources,curan::ui::VolumetricMask* in_vol_mas): resources{&in_resources},vol_mas{in_vol_mas}{}

    std::unique_ptr<curan::ui::Container> main_page();
};

enum Strategy{
    CONSERVATIVE,   
};

std::unique_ptr<curan::ui::Overlay> layout_overlay(Application& appdata);
std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata);
std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata);
std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page(Application& appdata);
void ac_pc_midline_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_ac_pc_midline(Application& appdata);
void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_target_and_region_of_entry(Application& appdata);
void select_entry_point_and_validate(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_entry_point_and_validate_point_selection(Application& appdata);
void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes);
std::unique_ptr<curan::ui::Container> select_roi_for_surgery(Application& appdata);

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

    BoundingBox(const BoundingBox& other) : origin{other.origin},orientation{other.orientation},size{other.size},spacing{other.spacing}{};
    
    BoundingBox centered_bounding_box(const Eigen::Matrix<double,3,3>& relative_transform){

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

        Eigen::Matrix<double,3,8> transformed_corners_in_rotated_space;
        for(size_t col = 0; col < static_cast<size_t>(transformed_corners_in_rotated_space.cols()); ++col)
            transformed_corners_in_rotated_space.col(col) = relative_transform.transpose()*corners_in_rotated_space.col(col);

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> minimum = transformed_corners_in_rotated_space.rowwise().minCoeff();

        //if(debug) std::cout << "\ndebug info: (minimum)\n" <<  transformed_corners_in_rotated_space.rowwise().minCoeff();

        transformed_corners_in_rotated_space.colwise() -=minimum;

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> unrounded_transformed_size = transformed_corners_in_rotated_space.rowwise().maxCoeff();

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
        transformed_rotation = orientation*relative_transform;

        auto transformed_quatered_bounding_box = transformed_rotation*(1.0/2.0)*transformed_corners_in_pixel_space;
        //if(debug) std::cout << "\ndebug info: (transformed_quatered_bounding_box)\n" <<  transformed_quatered_bounding_box;
        Eigen::Matrix<double,3,1> transformed_center_bounding_box = transformed_quatered_bounding_box.col(1)+transformed_quatered_bounding_box.col(2)+transformed_quatered_bounding_box.col(3);
        //if(debug) std::cout << "\ndebug info: (transformed_center_bounding_box)\n" <<  transformed_center_bounding_box;
        Eigen::Matrix<double,3,4> transformed_corners_in_world_space;
        transformed_corners_in_world_space.col(0) = origin+center_bounding_box-transformed_center_bounding_box;
        transformed_corners_in_world_space.col(1) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(1);
        transformed_corners_in_world_space.col(2) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(2);
        transformed_corners_in_world_space.col(3) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(3);

        //if(debug) std::cout << "\ndebug info: (transformed_corners_in_world_space)\n" <<  transformed_corners_in_world_space;

        Eigen::Matrix<double,3,4> corners_in_world_space;
        corners_in_world_space.col(0) = origin;
        corners_in_world_space.col(1) = corners_in_world_space.col(0)+orientation*corners_in_rotated_space.col(1);
        corners_in_world_space.col(2) = corners_in_world_space.col(0)+orientation*corners_in_rotated_space.col(2);
        corners_in_world_space.col(3) = corners_in_world_space.col(0)+orientation*corners_in_rotated_space.col(3);

        //if(debug) std::cout << "\ndebug info: (corners_in_world_space)\n" <<  corners_in_world_space;

        return BoundingBox{transformed_corners_in_world_space.col(0),transformed_corners_in_world_space.col(1),transformed_corners_in_world_space.col(2),transformed_corners_in_world_space.col(3),transformed_spacing};
    }
};

std::unique_ptr<curan::ui::Overlay> layout_overlay(Application& appdata)
{
    using namespace curan::ui;
    auto single_view_layout = Button::make(" ", "layout1x1.png", *appdata.resources);
    single_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    single_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { appdata.type = LayoutType::ONE; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto double_view_layout = Button::make(" ", "layout1x2.png", *appdata.resources);
    double_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    double_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { appdata.type = LayoutType::TWO; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto triple_view_layout = Button::make(" ", "layout1x3.png", *appdata.resources);
    triple_view_layout->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    triple_view_layout->add_press_call([&](Button *button, Press press, ConfigDraw *config)
    { appdata.type = LayoutType::THREE; appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK); });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(single_view_layout) << std::move(double_view_layout) << std::move(triple_view_layout);
    viwers_container->set_color(SK_ColorTRANSPARENT);
    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Container> create_dicom_viewers(Application& appdata){
    using namespace curan::ui;
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    switch(appdata.type){
        case ONE:
        {
            std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::X);
            *container << std::move(image_display);
        }
        break;
        case TWO:
        {
            std::unique_ptr<curan::ui::SlidingPanel> image_displayx = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::X);
            std::unique_ptr<curan::ui::SlidingPanel> image_displayy = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            *container << std::move(image_displayx) << std::move(image_displayy);
        }
        break;
        case THREE:
        {
            std::unique_ptr<curan::ui::SlidingPanel> image_displayx = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::X);
            std::unique_ptr<curan::ui::SlidingPanel> image_displayy = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::Y);
            std::unique_ptr<curan::ui::SlidingPanel> image_displayz = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::Z);
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

std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page(Application& appdata)
{
    using namespace curan::ui;
    using PixelType = unsigned char;
    auto item_explorer = ItemExplorer::make("file_icon.png", *appdata.resources);
    item_explorer->add_press_call([&](ItemExplorer *widget, Press press, ConfigDraw *draw){
            auto highlighted = widget->highlighted();
            assert(highlighted.size()==1 && "the size is larger than one");
            appdata.volume_index = highlighted.back();
            size_t i = 0;
            for(auto vol : appdata.volumes){
                if(i==appdata.volume_index)
                    appdata.vol_mas->update_volume(vol.second.img);
                ++i;
            }

    });
    using ImageType = itk::Image<PixelType, 3>;
    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;
    size_t identifier = 0;
    for (auto &vol : appdata.volumes)
    {
        if (vol.second.img.IsNotNull())
        {
            auto itk_pointer = vol.second.img;
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
            item_explorer->add(Item{identifier, vol.first + std::to_string(identifier), buff, extracted_size[0], extracted_size[1]});
        }
        ++identifier;
    }
    item_explorer->set_size(SkRect::MakeWH(800, 400));
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(item_explorer);
    return Overlay::make(std::move(container), SkColorSetARGB(100, 125, 125, 125), true);
}

void ac_pc_midline_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
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
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto defineac = Button::make("Locate AC", *appdata.resources);
    defineac->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    defineac->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = true;
        appdata.ac_pc_data.pc_specification = false;
    });

    appdata.ac_pc_data.ac_button = defineac.get();

    auto definepc = Button::make("Locate PC", *appdata.resources);
    definepc->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    definepc->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = true;
    });

    appdata.ac_pc_data.pc_button = definepc.get();

    auto resample = Button::make("Resample Volume", *appdata.resources);
    resample->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    resample->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = false;

        if(!appdata.ac_pc_data.ac_word_coordinates || !appdata.ac_pc_data.pc_word_coordinates){
            config->stack_page->stack(warning_overlay("must define AC-PC midline before resampling",*appdata.resources));
            return;
        }
        
        Eigen::Matrix<double, 3, 1> orient_along_ac_pc = *appdata.ac_pc_data.ac_word_coordinates - *appdata.ac_pc_data.pc_word_coordinates;
        if (orient_along_ac_pc.norm() < 1e-7){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("AC-PC line is singular, try different points",*appdata.resources));
            return;
        }
        orient_along_ac_pc.normalize(); 
        Eigen::Matrix<double, 3, 1> y_direction = orient_along_ac_pc;

        ImageType::Pointer input;
        if (auto search = appdata.volumes.find("source"); search != appdata.volumes.end())
            input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        auto direction = input->GetDirection();
        Eigen::Matrix<double, 3, 3> original_eigen_rotation_matrix;
        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                original_eigen_rotation_matrix(row, col) = direction(row, col);

        Eigen::Matrix<double,3,3> eigen_direction;
        for(size_t i = 0; i < 3; ++i)
            for(size_t j = 0;  j < 3; ++j)
                eigen_direction(i,j) = direction(i,j);


        Eigen::Matrix<double,3,1> x_direction = original_eigen_rotation_matrix.col(0);
        Eigen::Matrix<double,3,1> z_direction = x_direction.cross(y_direction);  
        x_direction = y_direction.cross(z_direction);  
        x_direction.normalize();
        std::cout << "projection: " << x_direction.transpose()*y_direction;
        z_direction = x_direction.cross(y_direction);  
        z_direction.normalize();
        if (z_direction.norm() < 1e-7){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("AC-PC line is singular when projected unto the axial plane, try different points",*appdata.resources));
            return;
        }
 
        Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
        eigen_rotation_matrix.col(0) = x_direction;
        eigen_rotation_matrix.col(1) = y_direction;
        eigen_rotation_matrix.col(2) = z_direction;

        if(original_eigen_rotation_matrix.determinant()<0.0)
            original_eigen_rotation_matrix.col(1) = -original_eigen_rotation_matrix.col(1);

        Eigen::Matrix<double, 3, 1> origin_for_bounding_box{{input->GetOrigin()[0], input->GetOrigin()[1], input->GetOrigin()[2]}};
        ImageType::PointType itk_along_dimension_x;
        ImageType::IndexType index_along_x{{(long long)input->GetLargestPossibleRegion().GetSize()[0], 0, 0}};
        input->TransformIndexToPhysicalPoint(index_along_x, itk_along_dimension_x);
        Eigen::Matrix<double, 3, 1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0], itk_along_dimension_x[1], itk_along_dimension_x[2]}};
        ImageType::PointType itk_along_dimension_y;
        ImageType::IndexType index_along_y{{0, (long long)input->GetLargestPossibleRegion().GetSize()[1], 0}};
        input->TransformIndexToPhysicalPoint(index_along_y, itk_along_dimension_y);
        Eigen::Matrix<double, 3, 1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0], itk_along_dimension_y[1], itk_along_dimension_y[2]}};
        ImageType::PointType itk_along_dimension_z;
        ImageType::IndexType index_along_z{{0, 0, (long long)input->GetLargestPossibleRegion().GetSize()[2]}};
        input->TransformIndexToPhysicalPoint(index_along_z, itk_along_dimension_z);
        Eigen::Matrix<double, 3, 1> extrema_along_z_for_bounding_box{{itk_along_dimension_z[0], itk_along_dimension_z[1], itk_along_dimension_z[2]}};
        Eigen::Matrix<double, 3, 1> spacing{{input->GetSpacing()[0], input->GetSpacing()[1], input->GetSpacing()[2]}};

        BoundingBox bounding_box_original_image{origin_for_bounding_box, extrema_along_x_for_bounding_box, extrema_along_y_for_bounding_box, extrema_along_z_for_bounding_box, spacing};
        auto output_bounding_box = bounding_box_original_image.centered_bounding_box(original_eigen_rotation_matrix.transpose() * eigen_rotation_matrix);
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

        filter->SetOutputDirection(rotation_matrix);

        try{
            filter->Update();
            auto output = filter->GetOutput();
            if (config->stack_page != nullptr) {
                config->stack_page->replace_last(success_overlay("resampled volume!",*appdata.resources));
            }
            appdata.volumes.emplace("acpc",output);
        } catch (...){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("failed to resample volume to AC-PC",*appdata.resources));
        }
    });

    auto switch_volume = Button::make("Switch Volume", *appdata.resources);
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    switch_volume->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.ac_pc_data.ac_specification = false;
        appdata.ac_pc_data.pc_specification = false;
        if(config->stack_page!=nullptr){
			config->stack_page->stack(create_volume_explorer_page(appdata));
		}
    });

    auto check = Button::make("Check", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.panel_constructor = select_target_and_region_of_entry;
        appdata.volume_callback = select_target_and_region_of_entry_point_selection;
        if(appdata.ac_pc_data.ac_word_coordinates && appdata.ac_pc_data.pc_word_coordinates)
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        else
            config->stack_page->replace_last(warning_overlay("cannot advance without AC-PC specification",*appdata.resources));
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(defineac) << std::move(definepc) << std::move(resample) << std::move(switch_volume) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

void select_target_and_region_of_entry_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
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

        auto compute = [&](Eigen::Matrix<double,3,1> world_coordinates){
            ImageType::IndexType point_index;
            ImageType::PointType point_world_coordinates;     
            point_world_coordinates[0] = world_coordinates[0];
            point_world_coordinates[1] = world_coordinates[1];
            point_world_coordinates[2] = world_coordinates[2];
            vol_mas->get_volume()->TransformPhysicalPointToIndex(point_world_coordinates,point_index);
            Eigen::Matrix<double,3,1> normalized_itk_points;
            normalized_itk_points << point_index[0]/(double)vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[0] 
                                   , point_index[1]/(double)vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[1] 
                                   , point_index[2]/(double)vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[2];
            return normalized_itk_points;
        };
        const auto target_local_index = compute(*appdata.trajectory_location.target_world_coordinates);
        const auto main_diagonal_local_index = compute(*appdata.trajectory_location.main_diagonal_word_coordinates);
       

        Eigen::Matrix<double,3,1> vector_aligned = target_local_index-main_diagonal_local_index;
        Eigen::Matrix<double,4,4> offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
        vector_aligned.normalize();
        Eigen::Matrix<double,3,1> yAxis(0, 0, 1);
        Eigen::Matrix<double,3,1> axis = yAxis.cross(vector_aligned);
        axis.normalize();
        double angle = std::acos(yAxis.transpose()*vector_aligned);
        auto final_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        vector_aligned = target_local_index-main_diagonal_local_index;
        double scaling_along_z = vector_aligned.norm();
        double scaling_along_x = std::tan(0.349066)*scaling_along_z;
        offset_base_to_Oxy(0,0) = scaling_along_x;
        offset_base_to_Oxy(1,1) = scaling_along_x;
        offset_base_to_Oxy(2,2) = scaling_along_z;
        geom.transform(offset_base_to_Oxy);
        offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
        offset_base_to_Oxy.block<3,3>(0,0) = final_rotation;
        geom.transform(offset_base_to_Oxy);
        auto tip = geom.geometry.vertices[0];
        Eigen::Matrix<double,3,1> translation;
        translation << (double)tip[0] , (double)tip[1] , (double)tip[2];
        offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
        offset_base_to_Oxy.block<3,1>(0,3) = -translation;  
        geom.transform(offset_base_to_Oxy);
        offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
        offset_base_to_Oxy.block<3,1>(0,3) = target_local_index;          
        geom.transform(offset_base_to_Oxy);
        appdata.vol_mas->add_geometry(geom);  
        
        for(size_t i = 0; i < geom.geometry.vertices.size(); ++i){
            ImageType::IndexType local_index;
            ImageType::PointType itk_point_in_world_coordinates;
            local_index[0] = vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[0]*(double)geom.geometry.vertices[i][0];
            local_index[1] = vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[1]*(double)geom.geometry.vertices[i][1];
            local_index[2] = vol_mas->get_volume()->GetLargestPossibleRegion().GetSize()[2]*(double)geom.geometry.vertices[i][2];
            vol_mas->get_volume()->TransformIndexToPhysicalPoint(local_index, itk_point_in_world_coordinates);
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][0] = itk_point_in_world_coordinates[0];
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][1] = itk_point_in_world_coordinates[1];
            appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[i][2] = itk_point_in_world_coordinates[2];
        }
    }

}

std::unique_ptr<curan::ui::Container> select_target_and_region_of_entry(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto definetarget = Button::make("Define Target", *appdata.resources);
    definetarget->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    definetarget->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = true;        
    });

    appdata.trajectory_location.target_button = definetarget.get();

    auto defineentryregion = Button::make("Define Entry Region", *appdata.resources);
    defineentryregion->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    defineentryregion->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = true;
        appdata.trajectory_location.target_specification = false;
    });

    appdata.trajectory_location.main_diagonal_button = defineentryregion.get();

    auto validadetrajectory = Button::make("Validade Trajectory", *appdata.resources);
    validadetrajectory->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    validadetrajectory->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;

        if(!appdata.trajectory_location.main_diagonal_word_coordinates || !appdata.trajectory_location.target_world_coordinates){
            config->stack_page->stack(warning_overlay("both the target and the entry region must be selected",*appdata.resources));
            return;
        }
        auto stored_geometries = appdata.vol_mas->geometries();  
        auto piramid = stored_geometries.front();
        auto convert_to_eigen = [](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
            Eigen::Matrix<double,3,1> converted;
            converted << point[0] , point[1] , point[2];
            return converted;
        };

        auto tip = convert_to_eigen(piramid.geometry.vertices[0]);
        auto base0 = convert_to_eigen(piramid.geometry.vertices[1]);
        auto base1 = convert_to_eigen(piramid.geometry.vertices[2]);
        auto base2 = convert_to_eigen(piramid.geometry.vertices[3]);
        auto base3 = convert_to_eigen(piramid.geometry.vertices[4]);

        auto average = 0.25*base0+0.25*base1+0.25*base2+0.25*base3;

        Eigen::Matrix<double, 3, 1> orient_along_traj = tip-average;
        if (orient_along_traj.norm() < 1e-7){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("Geometry is compromised",*appdata.resources));
            return;
        }
        orient_along_traj.normalize(); 
        Eigen::Matrix<double, 3, 1> x_direction = orient_along_traj;

        ImageType::Pointer input;
        if (auto search = appdata.volumes.find("source"); search != appdata.volumes.end())
            input = search->second.img;
        else{
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        auto direction = input->GetDirection();
        Eigen::Matrix<double, 3, 3> original_eigen_rotation_matrix;
        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                original_eigen_rotation_matrix(row, col) = direction(row, col);

        Eigen::Matrix<double,3,3> eigen_direction;
        for(size_t i = 0; i < 3; ++i)
            for(size_t j = 0;  j < 3; ++j)
                eigen_direction(i,j) = direction(i,j);


        Eigen::Matrix<double,3,1> y_direction = base3 - base2;
        Eigen::Matrix<double,3,1> z_direction = base3 - base0;
        if (z_direction.norm() < 1e-7 || y_direction.norm() < 1e-7){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("Geometry is compromised",*appdata.resources));
            return;
        }
        y_direction.normalize();
        z_direction.normalize();
        Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
        eigen_rotation_matrix.col(0) = x_direction;
        eigen_rotation_matrix.col(1) = y_direction;
        eigen_rotation_matrix.col(2) = z_direction;

        if(original_eigen_rotation_matrix.determinant()<0.0)
            original_eigen_rotation_matrix.col(1) = -original_eigen_rotation_matrix.col(1);

        Eigen::Matrix<double, 3, 1> origin_for_bounding_box{{input->GetOrigin()[0], input->GetOrigin()[1], input->GetOrigin()[2]}};
        ImageType::PointType itk_along_dimension_x;
        ImageType::IndexType index_along_x{{(long long)input->GetLargestPossibleRegion().GetSize()[0], 0, 0}};
        input->TransformIndexToPhysicalPoint(index_along_x, itk_along_dimension_x);
        Eigen::Matrix<double, 3, 1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0], itk_along_dimension_x[1], itk_along_dimension_x[2]}};
        ImageType::PointType itk_along_dimension_y;
        ImageType::IndexType index_along_y{{0, (long long)input->GetLargestPossibleRegion().GetSize()[1], 0}};
        input->TransformIndexToPhysicalPoint(index_along_y, itk_along_dimension_y);
        Eigen::Matrix<double, 3, 1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0], itk_along_dimension_y[1], itk_along_dimension_y[2]}};
        ImageType::PointType itk_along_dimension_z;
        ImageType::IndexType index_along_z{{0, 0, (long long)input->GetLargestPossibleRegion().GetSize()[2]}};
        input->TransformIndexToPhysicalPoint(index_along_z, itk_along_dimension_z);
        Eigen::Matrix<double, 3, 1> extrema_along_z_for_bounding_box{{itk_along_dimension_z[0], itk_along_dimension_z[1], itk_along_dimension_z[2]}};
        Eigen::Matrix<double, 3, 1> spacing{{input->GetSpacing()[0], input->GetSpacing()[1], input->GetSpacing()[2]}};

        BoundingBox bounding_box_original_image{origin_for_bounding_box, extrema_along_x_for_bounding_box, extrema_along_y_for_bounding_box, extrema_along_z_for_bounding_box, spacing};
        auto output_bounding_box = bounding_box_original_image.centered_bounding_box(original_eigen_rotation_matrix.transpose() * eigen_rotation_matrix);
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

        filter->SetOutputDirection(rotation_matrix);

        try{
            filter->Update();
            auto output = filter->GetOutput();
            appdata.volumes.emplace("trajectory",output);
            if (config->stack_page != nullptr) {
                config->stack_page->replace_last(success_overlay("resampled volume!",*appdata.resources));
            }
        } catch (...){
            if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("failed to resample trajectory volume",*appdata.resources));
        }

        appdata.panel_constructor = select_entry_point_and_validate_point_selection;
        appdata.volume_callback = std::function<void(Application&,curan::ui::VolumetricMask*, curan::ui::ConfigDraw*, const curan::ui::directed_stroke&)>{};
        appdata.projected_volume_callback = select_entry_point_and_validate;
        appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
    });

    auto switch_volume = Button::make("Switch Volume", *appdata.resources);
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    switch_volume->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;
        if(config->stack_page!=nullptr){
			config->stack_page->stack(create_volume_explorer_page(appdata));
		}
    });

    auto check = Button::make("Check", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;

        if(appdata.trajectory_location.main_diagonal_word_coordinates && appdata.trajectory_location.target_world_coordinates && appdata.trajectory_location.entry_point_word_coordinates)
            appdata.tradable_page->construct(appdata.panel_constructor(appdata),SK_ColorBLACK);
        else
            config->stack_page->replace_last(warning_overlay("cannot advance without trajectory specified",*appdata.resources));
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(definetarget) << std::move(defineentryregion) << std::move(validadetrajectory) << std::move(switch_volume) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

void select_entry_point_and_validate(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
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

    if(appdata.trajectory_location.entry_specification){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.entry_point_word_coordinates = word_coordinates;

        ImageType::Pointer input;
        if (auto search = appdata.volumes.find("source"); search != appdata.volumes.end())
            input = search->second.img;
        else{
            if (config_draw->stack_page != nullptr) config_draw->stack_page->replace_last(warning_overlay("could not find source dicom image",*appdata.resources));
            return;
        }

        input->TransformPhysicalPointToIndex(itk_point_in_world_coordinates,local_index);
        Eigen::Matrix<double, 3, 1> entry_index_coordinates = Eigen::Matrix<double, 3, 1>::Zero();
        entry_index_coordinates[0] = local_index[0];
        entry_index_coordinates[1] = local_index[1];
        entry_index_coordinates[2] = local_index[2];

        config_draw->stack_page->stack(success_overlay("entry defined",*appdata.resources));

        auto convert_to_eigen = [&](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
            ImageType::IndexType local_index;
            ImageType::PointType itk_point_in_world_coordinates;
            itk_point_in_world_coordinates[0] = (double)point[0];
            itk_point_in_world_coordinates[1] = (double)point[1];
            itk_point_in_world_coordinates[2] = (double)point[2];
            input->TransformPhysicalPointToIndex(itk_point_in_world_coordinates,local_index);
            Eigen::Matrix<double,3,1> converted;
            converted[0] = local_index[0];
            converted[1] = local_index[1];
            converted[2] = local_index[2];
            converted << point[0] , point[1] , point[2];
            return converted;
        };

        auto tip = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[0]);
        auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
        auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);
        auto base2 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[3]);
        auto base3 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[4]);

        Eigen::Matrix<double, 3, 1> orient_along_traj = tip-entry_index_coordinates;
        if (orient_along_traj.norm() < 1e-7){
            if (config_draw->stack_page != nullptr) config_draw->stack_page->replace_last(warning_overlay("Geometry is compromised",*appdata.resources));
            return;
        }
        orient_along_traj.normalize(); 
        Eigen::Matrix<double, 3, 1> x_direction = orient_along_traj;

        auto direction = input->GetDirection();
        Eigen::Matrix<double, 3, 3> original_eigen_rotation_matrix;
        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                original_eigen_rotation_matrix(row, col) = direction(row, col);

        Eigen::Matrix<double,3,3> eigen_direction;
        for(size_t i = 0; i < 3; ++i)
            for(size_t j = 0;  j < 3; ++j)
                eigen_direction(i,j) = direction(i,j);

        Eigen::Matrix<double,3,1> y_direction = base3 - base2;
        Eigen::Matrix<double,3,1> z_direction = x_direction.cross(y_direction);
        z_direction.normalize();
        y_direction = z_direction.cross(x_direction);
        y_direction.normalize();
        z_direction = x_direction.cross(y_direction);
        z_direction.normalize();

        if (y_direction.norm() < 1e-7){
            if (config_draw->stack_page != nullptr) config_draw->stack_page->replace_last(warning_overlay("Geometry is compromised",*appdata.resources));
            return;
        }
        Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
        eigen_rotation_matrix.col(0) = x_direction;
        eigen_rotation_matrix.col(1) = y_direction;
        eigen_rotation_matrix.col(2) = z_direction;

        if(original_eigen_rotation_matrix.determinant()<0.0)
            original_eigen_rotation_matrix.col(1) = -original_eigen_rotation_matrix.col(1);

        Eigen::Matrix<double, 3, 1> origin_for_bounding_box{{input->GetOrigin()[0], input->GetOrigin()[1], input->GetOrigin()[2]}};
        ImageType::PointType itk_along_dimension_x;
        ImageType::IndexType index_along_x{{(long long)input->GetLargestPossibleRegion().GetSize()[0], 0, 0}};
        input->TransformIndexToPhysicalPoint(index_along_x, itk_along_dimension_x);
        Eigen::Matrix<double, 3, 1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0], itk_along_dimension_x[1], itk_along_dimension_x[2]}};
        ImageType::PointType itk_along_dimension_y;
        ImageType::IndexType index_along_y{{0, (long long)input->GetLargestPossibleRegion().GetSize()[1], 0}};
        input->TransformIndexToPhysicalPoint(index_along_y, itk_along_dimension_y);
        Eigen::Matrix<double, 3, 1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0], itk_along_dimension_y[1], itk_along_dimension_y[2]}};
        ImageType::PointType itk_along_dimension_z;
        ImageType::IndexType index_along_z{{0, 0, (long long)input->GetLargestPossibleRegion().GetSize()[2]}};
        input->TransformIndexToPhysicalPoint(index_along_z, itk_along_dimension_z);
        Eigen::Matrix<double, 3, 1> extrema_along_z_for_bounding_box{{itk_along_dimension_z[0], itk_along_dimension_z[1], itk_along_dimension_z[2]}};
        Eigen::Matrix<double, 3, 1> spacing{{input->GetSpacing()[0], input->GetSpacing()[1], input->GetSpacing()[2]}};

        BoundingBox bounding_box_original_image{origin_for_bounding_box, extrema_along_x_for_bounding_box, extrema_along_y_for_bounding_box, extrema_along_z_for_bounding_box, spacing};
        auto output_bounding_box = bounding_box_original_image.centered_bounding_box(original_eigen_rotation_matrix.transpose() * eigen_rotation_matrix);
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

        filter->SetOutputDirection(rotation_matrix);

        try{
            filter->Update();
            auto output = filter->GetOutput();
            appdata.vol_mas->update_volume(output);
            curan::geometry::Piramid geom{curan::geometry::CENTROID_ALIGNED};
            // now I need the tip and the entry point from world coordinates into local coordinates
            
            Eigen::Matrix<double,3,1> vector_aligned = x_direction;
            Eigen::Matrix<double,4,4> offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            vector_aligned.normalize();
            Eigen::Matrix<double,3,1> yAxis(0, 0, 1);
            Eigen::Matrix<double,3,1> axis = yAxis.cross(vector_aligned);
            axis.normalize();
            double angle = std::acos(yAxis.transpose()*vector_aligned);
            auto final_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
            vector_aligned = tip-entry_index_coordinates;
            double scaling_along_z = vector_aligned.norm();
            double scaling_along_x = std::tan(0.1)*scaling_along_z;
            offset_base_to_Oxy(0,0) = scaling_along_x;
            offset_base_to_Oxy(1,1) = scaling_along_x;
            offset_base_to_Oxy(2,2) = scaling_along_z;
            geom.transform(offset_base_to_Oxy);
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy.block<3,3>(0,0) = final_rotation;
            geom.transform(offset_base_to_Oxy);
            auto localtip = geom.geometry.vertices[0];
            Eigen::Matrix<double,3,1> translation;
            translation << (double)localtip[0] , (double)localtip[1] , (double)localtip[2];
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy.block<3,1>(0,3) = -translation;  
            geom.transform(offset_base_to_Oxy);
            offset_base_to_Oxy = Eigen::Matrix<double,4,4>::Identity();
            offset_base_to_Oxy.block<3,1>(0,3) = tip;          
            geom.transform(offset_base_to_Oxy);
            appdata.vol_mas->add_geometry(geom);  
            std::cout << "geometry: " << geom << std::endl;
            if (config_draw->stack_page != nullptr) {
                config_draw->stack_page->replace_last(success_overlay("resampled volume!",*appdata.resources));
            }

        } catch (...){
            if (config_draw->stack_page != nullptr) config_draw->stack_page->replace_last(warning_overlay("failed to resample trajectory volume",*appdata.resources));
        }

    }

}

ImageType::Pointer allocate_image(Application& appdata){

    ImageType::Pointer input;
    if (auto search = appdata.volumes.find("source"); search != appdata.volumes.end())
        input = search->second.img;
    else{
        return nullptr;
    }

    auto convert_to_eigen = [&](gte::Vector3<curan::geometry::PolyHeadra::Rational> point){
        ImageType::IndexType local_index;
        ImageType::PointType itk_point_in_world_coordinates;
        itk_point_in_world_coordinates[0] = (double)point[0];
        itk_point_in_world_coordinates[1] = (double)point[1];
        itk_point_in_world_coordinates[2] = (double)point[2];
        input->TransformPhysicalPointToIndex(itk_point_in_world_coordinates,local_index);
        Eigen::Matrix<double,3,1> converted;
        converted << (double)local_index[0] , (double)local_index[1], (double)local_index[2];
        return converted;
    };

    auto tip = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[0]);
    auto base0 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[1]);
    auto base1 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[2]);
    auto base2 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[3]);
    auto base3 = convert_to_eigen(appdata.trajectory_location.piramid_world_coordinates.geometry.vertices[4]);

    auto average = 0.25*base0+0.25*base1+0.25*base2+0.25*base3;

    Eigen::Matrix<double, 3, 1> orient_along_traj = tip-average;
    orient_along_traj.normalize(); 
    Eigen::Matrix<double, 3, 1> x_direction = orient_along_traj;

    auto direction = input->GetDirection();
    Eigen::Matrix<double,3,3> eigen_direction;
    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0;  j < 3; ++j)
            eigen_direction(i,j) = direction(i,j);


    Eigen::Matrix<double,3,1> y_direction = base3 - base2;
    Eigen::Matrix<double,3,1> z_direction = base3 - base0;
    y_direction.normalize();
    z_direction.normalize();
    Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
    eigen_rotation_matrix.col(0) = x_direction;
    eigen_rotation_matrix.col(1) = y_direction;
    eigen_rotation_matrix.col(2) = z_direction;

    ImageType::Pointer image = ImageType::New();
  
    ImageType::SizeType size;
    size[0] = 512;  // size along X
    size[1] = 512;  // size along Y 
    size[2] = 1;   // size along Z

    double minspacing = std::min((base3 - base2).norm()/(double)size[0],(base3 - base0).norm()/(double)size[1]);
  
    ImageType::SpacingType spacing;
    spacing[0] = minspacing; // mm along X
    spacing[1] = minspacing; // mm along X
    spacing[2] = minspacing; // mm along Z
  
    ImageType::PointType origin;
    origin[0] = average[0];
    origin[1] = average[1];
    origin[2] = average[2];
  
    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            direction(i,j) = eigen_rotation_matrix(i,j);
  
    ImageType::RegionType region;
    region.SetSize(size);
  
    image->SetRegions(region);
    image->SetSpacing(spacing);
    image->SetOrigin(origin);
    image->SetDirection(direction);
    image->Allocate();
    image->FillBuffer(0);

    typedef itk::ImageRegionIteratorWithIndex<ImageType> IteratorType;
    IteratorType it(image, image->GetRequestedRegion());

    Eigen::Matrix<double,3,1> t0 = tip;
    Eigen::Matrix<double,3,1> texcoord;
    Eigen::Matrix<double,3,1> deltaTexCoord;
    Eigen::Matrix<double,3,1> te;

    ImageType::IndexType tmpindex;
    tmpindex[0] = 0;
    tmpindex[1] = 0;
    tmpindex[2] = 0;

    auto overall_size = input->GetLargestPossibleRegion().GetSize();

    for (it.GoToBegin(); !it.IsAtEnd(); ++it)
    {
        ImageType::IndexType index = it.GetIndex();
        te[0] = index[0];
        te[1] = index[1];
        te[2] = index[2];

        const float min_iteratrions = 2.0;
        const float max_iteratrions = 1024.0;

        const float TransparencyValue = 0.02;
        const float AlphaFuncValue = 0.025;
        const float SampleDensityValue = 0.05;
        

        float num_iterations = ceil((te-t0).norm()/SampleDensityValue);
        if (num_iterations<min_iteratrions) num_iterations = min_iteratrions;
        else if (num_iterations>max_iteratrions) num_iterations = max_iteratrions;

        auto mix = [](Eigen::Matrix<double,4,1> l, Eigen::Matrix<double,4,1> r,double mix_ratio){
            Eigen::Matrix<double,3,1> mixed;
            mixed = l.block<3,1>(0,0)*(1.0-mix_ratio)+r.block<3,1>(0,0)*mix_ratio;
            return mixed;
        };

        deltaTexCoord = (te-t0)/(num_iterations-1.0);
        texcoord = t0;
        Eigen::Matrix<double,4,1> fragColor = Eigen::Matrix<double,4,1>::Zero();
        tmpindex[0] = texcoord[0];
        tmpindex[1] = texcoord[1];
        tmpindex[2] = texcoord[2];
        float alpha = (1.0/255.0)*input->GetPixel(tmpindex);
        fragColor << alpha, alpha, alpha, alpha * TransparencyValue;
        while(num_iterations>0.0){
            tmpindex[0] = texcoord[0];
            tmpindex[1] = texcoord[1];
            tmpindex[2] = texcoord[2];
            if(tmpindex[0] < 0                || tmpindex[1] < 0                || tmpindex[2] < 0 || 
               tmpindex[0] >= overall_size[0] || tmpindex[1] >= overall_size[1] || tmpindex[2] >= overall_size[2])
                break;
            alpha = (1.0/255.0)*input->GetPixel(tmpindex);
            //alpha = 1.0-alpha;
            Eigen::Matrix<double,4,1> color;
            color << alpha, alpha, alpha, alpha * TransparencyValue;
            float r = color[3];
            if (r > AlphaFuncValue){
                fragColor.block<3,1>(0,0) = mix(fragColor, color, r);
                fragColor[3] += r;
            }

            if (color[3] > fragColor[3])
                fragColor = color;

            texcoord += deltaTexCoord;
            --num_iterations;
        }
        it.Set((255.0)*(1.0/3.0)*(fragColor[0]+fragColor[1]+fragColor[2]));
  }


  return image;
}

std::unique_ptr<curan::ui::Container> select_entry_point_and_validate_point_selection(Application& appdata){
    using namespace curan::ui;

    ImageType::Pointer input;
    if (auto search = appdata.volumes.find("trajectory"); search != appdata.volumes.end())
        input = search->second.img;
    else{
        throw std::runtime_error("failure due to missing volume");
    }
    appdata.vol_mas->update_volume(input);
    ImageType::Pointer projected_input = allocate_image(appdata);
    appdata.projected_vol_mas.update_volume(projected_input);

    auto slidercontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    std::unique_ptr<curan::ui::SlidingPanel> image_displayx = curan::ui::SlidingPanel::make(*appdata.resources, &appdata.projected_vol_mas, Direction::Z);
    std::unique_ptr<curan::ui::SlidingPanel> image_displayy = curan::ui::SlidingPanel::make(*appdata.resources, appdata.vol_mas, Direction::X);
    *slidercontainer << std::move(image_displayx) << std::move(image_displayy);

    auto defineentry = Button::make("Select Entry Point", *appdata.resources);
    defineentry->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    defineentry->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = true;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;    
    });

    auto check = Button::make("Check", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    check->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        appdata.trajectory_location.entry_specification = false;
        appdata.trajectory_location.main_diagonal_specification = false;
        appdata.trajectory_location.target_specification = false;
    });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(defineentry) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(slidercontainer);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
}

void select_roi_for_surgery_point_selection(Application& appdata,curan::ui::VolumetricMask *vol_mas, curan::ui::ConfigDraw *config_draw, const curan::ui::directed_stroke &strokes){
    // now we need to convert between itk coordinates and real world coordinates
    if (!(strokes.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if (config_draw && config_draw->stack_page && strokes.point_in_image_coordinates.cols() > 1)
    {
        config_draw->stack_page->stack(warning_overlay("must select a single point, not a path",*appdata.resources));
        return;
    }
}

std::unique_ptr<curan::ui::Container> select_roi_for_surgery(Application& appdata){
    using namespace curan::ui;

    auto image_display = create_dicom_viewers(appdata);
    auto layout = Button::make("Layout", *appdata.resources);
    layout->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    layout->add_press_call([&](Button *button, Press press, ConfigDraw *config){
        if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay(appdata));
		}
    });

    auto addroi = Button::make("Add ROI", *appdata.resources);
    addroi->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));

    auto switch_volume = Button::make("Switch Volume", *appdata.resources);
    switch_volume->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));

    auto check = Button::make("Check", *appdata.resources);
    check->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(layout) << std::move(addroi) << std::move(switch_volume) << std::move(check);

    viwers_container->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    return std::move(container);
};

std::unique_ptr<curan::ui::Container> Application::main_page(){
    using namespace curan::ui;
    panel_constructor = select_ac_pc_midline;
    volume_callback = ac_pc_midline_point_selection;
    auto container_with_widgets = select_ac_pc_midline(*this);
    std::unique_ptr<MiniPage> minipage = MiniPage::make(std::move(container_with_widgets), SK_ColorBLACK);
    tradable_page = minipage.get();
    auto minimage_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *minimage_container << std::move(minipage);
    vol_mas->add_pressedhighlighted_call([this](VolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
        volume_callback(*this,vol_mas,config_draw,strokes);
    });
    projected_vol_mas.add_pressedhighlighted_call([this](VolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes){
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


int main(int argc, char* argv[]) {
	using namespace curan::ui;
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context)};
	param.windowName = "Curan:Path Planner";
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

    using ImageReaderType = itk::ImageFileReader<itk::Image<double,3>>;

    std::printf("\nReading input volume...\n");
    auto fixedImageReader = ImageReaderType::New();
    fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH"/precious_phantom/johndoe.mha");

    auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
    rescale->SetInput(fixedImageReader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(255.0);

    auto castfilter = itk::CastImageFilter<itk::Image<double,3>, ImageType>::New();
    castfilter->SetInput(rescale->GetOutput());

    itk::OrientImageFilter<ImageType,ImageType>::Pointer orienter =itk::OrientImageFilter<ImageType,ImageType>::New();
    orienter->UseImageDirectionOn();
    orienter->SetDesiredCoordinateOrientation(itk::SpatialOrientation::ITK_COORDINATE_ORIENTATION_RAS);
     
    orienter->SetInput(castfilter->GetOutput());

    orienter->Update();
    
    VolumetricMask vol{orienter->GetOutput()};
    Application appdata{resources,&vol};
    appdata.volumes.emplace("source",orienter->GetOutput());
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
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	return 0;
}