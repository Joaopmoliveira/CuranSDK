#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/MiniPage.h"
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

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#include <Eigen/Dense>

enum Strategy{
    CONSERVATIVE,   
};

struct BoundingBox{
    Eigen::Matrix<double,3,1> origin;
    Eigen::Matrix<double,3,3> orientation;
    Eigen::Matrix<double,3,1> size;
    Eigen::Matrix<double,3,1> spacing;

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

        assert(determinant>0.9999 && determinant<1.0001 && "failure to generate an ortogonal rotation matrix");
        spacing = in_spacing;
        size[0] = vector_along_direction_x.norm()/spacing[0];
        size[1] = vector_along_direction_y.norm()/spacing[1];
        size[2] = vector_along_direction_z.norm()/spacing[2];
    }

    friend std::ostream & operator << (std::ostream &, const BoundingBox &);

    template<Strategy prefered_strategy,bool debug>
    BoundingBox centered_bounding_box(const Eigen::Matrix<double,3,3>& relative_transform){

        if(debug) std::cout << "\ndebug info: (relative_transform)\n" <<  relative_transform;

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

        if(debug) std::cout << "\ndebug info: (corners_in_rotated_space)\n" <<  corners_in_rotated_space;

        Eigen::Matrix<double,3,8> transformed_corners_in_rotated_space;
        for(size_t col = 0; col < static_cast<size_t>(transformed_corners_in_rotated_space.cols()); ++col)
            transformed_corners_in_rotated_space.col(col) = relative_transform.transpose()*corners_in_rotated_space.col(col);

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> minimum = transformed_corners_in_rotated_space.rowwise().minCoeff();

        if(debug) std::cout << "\ndebug info: (minimum)\n" <<  transformed_corners_in_rotated_space.rowwise().minCoeff();

        transformed_corners_in_rotated_space.colwise() -=minimum;

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> unrounded_transformed_size = transformed_corners_in_rotated_space.rowwise().maxCoeff();

        if(debug) std::cout << "\ndebug info: (unrounded_transformed_size)\n" <<  unrounded_transformed_size;

        Eigen::Matrix<double,3,1> transformed_spacing = spacing;
        transformed_spacing.fill(spacing.minCoeff());

        if(debug) std::cout << "\ndebug info: (transformed_spacing)\n" <<  transformed_spacing;

        Eigen::Matrix<double,3,1> transformed_size;
        transformed_size[0] = std::ceil(unrounded_transformed_size[0]);
        transformed_size[1] = std::ceil(unrounded_transformed_size[1]);
        transformed_size[2] = std::ceil(unrounded_transformed_size[2]);

        if(debug) std::cout << "\ndebug info: (transformed_size)\n" <<  transformed_size;

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

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_pixel_space)\n" <<  transformed_corners_in_pixel_space;

        auto quatered_bounding_box = orientation*(1.0/2.0)*corners_in_rotated_space;
        if(debug) std::cout << "\ndebug info: (quatered_bounding_box)\n" <<  quatered_bounding_box;
        Eigen::Matrix<double,3,1> center_bounding_box = quatered_bounding_box.col(1)+quatered_bounding_box.col(2)+quatered_bounding_box.col(3);
        if(debug) std::cout << "\ndebug info: (center_bounding_box)\n" <<  center_bounding_box;

        Eigen::Matrix<double,3,3> transformed_rotation;
        transformed_rotation = orientation*relative_transform;

        auto transformed_quatered_bounding_box = transformed_rotation*(1.0/2.0)*transformed_corners_in_pixel_space;
        if(debug) std::cout << "\ndebug info: (transformed_quatered_bounding_box)\n" <<  transformed_quatered_bounding_box;
        Eigen::Matrix<double,3,1> transformed_center_bounding_box = transformed_quatered_bounding_box.col(1)+transformed_quatered_bounding_box.col(2)+transformed_quatered_bounding_box.col(3);
        if(debug) std::cout << "\ndebug info: (transformed_center_bounding_box)\n" <<  transformed_center_bounding_box;
        Eigen::Matrix<double,3,4> transformed_corners_in_world_space;
        transformed_corners_in_world_space.col(0) = origin+center_bounding_box-transformed_center_bounding_box;
        transformed_corners_in_world_space.col(1) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(1);
        transformed_corners_in_world_space.col(2) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(2);
        transformed_corners_in_world_space.col(3) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(3);

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_world_space)\n" <<  transformed_corners_in_world_space;

        return BoundingBox{transformed_corners_in_world_space.col(0),transformed_corners_in_world_space.col(1),transformed_corners_in_world_space.col(2),transformed_corners_in_world_space.col(3),transformed_spacing};
    }
};

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

enum Panels
{
    ONE_PANEL,
    TWO_PANELS,
    THREE_PANELS
};

enum PanelType
{
    ORIGINAL_VOLUME = 0,
    RESAMPLED_VOLUME,
    TRAJECTORY_ORIENTED_VOLUME,
    NUMBER_OF_VOLUMES
};

struct DataSpecificApplication
{
    bool is_acpc_being_defined = false;
    bool is_trajectory_being_visualized = false;

    std::array<curan::ui::VolumetricMask, PanelType::NUMBER_OF_VOLUMES> map;
    PanelType current_volume = PanelType::ORIGINAL_VOLUME;

    curan::ui::IconResources &resources;

    Panels current_panel_arragement = Panels::ONE_PANEL;

    std::optional<Eigen::Matrix<double, 3, 1>> ac_point;
    std::optional<Eigen::Matrix<double, 3, 1>> pc_point;
    std::optional<Eigen::Matrix<double, 3, 1>> midline;

    curan::ui::MiniPage *minipage = nullptr;

    DataSpecificApplication(ImageType::Pointer volume, curan::ui::IconResources &in_resources) : resources{in_resources}, map{{{volume}, {volume}, {volume}}}
    {
    }

    std::unique_ptr<curan::ui::Overlay> create_overlay_with_warning(const std::string &warning)
    {
        using namespace curan::ui;
        auto warn = Button::make(" ", "warning.png", resources);
        warn->set_click_color(SK_AlphaTRANSPARENT).set_hover_color(SK_AlphaTRANSPARENT).set_waiting_color(SK_AlphaTRANSPARENT).set_size(SkRect::MakeWH(400, 200));

        auto button = Button::make(warning, resources);
        button->set_click_color(SK_AlphaTRANSPARENT).set_hover_color(SK_AlphaTRANSPARENT).set_waiting_color(SK_AlphaTRANSPARENT).set_size(SkRect::MakeWH(200, 50));

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        *viwers_container << std::move(warn) << std::move(button);
        viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0, .8, 1.0});

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Overlay> create_overlay_with_success(const std::string &success)
    {
        using namespace curan::ui;
        auto warn = Button::make(" ", "submit.png", resources);
        warn->set_click_color(SK_AlphaTRANSPARENT).set_hover_color(SK_AlphaTRANSPARENT).set_waiting_color(SK_AlphaTRANSPARENT).set_size(SkRect::MakeWH(400, 200));

        auto button = Button::make(success, resources);
        button->set_click_color(SK_AlphaTRANSPARENT).set_hover_color(SK_AlphaTRANSPARENT).set_waiting_color(SK_AlphaTRANSPARENT).set_size(SkRect::MakeWH(200, 50));

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        *viwers_container << std::move(warn) << std::move(button);
        viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0, .8, 1.0});

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    void create_panel_ac_pc_instructions()
    {
        using namespace curan::ui;
        if (!is_acpc_being_defined)
        {
            auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);

            switch (current_panel_arragement)
            {
            case Panels::ONE_PANEL:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                *viwers_container << std::move(image_display);
            }
            break;
            case Panels::TWO_PANELS:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Y);
                *viwers_container << std::move(image_display_x) << std::move(image_display_y);
            }
            break;
            case Panels::THREE_PANELS:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Y);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_z = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Z);
                *viwers_container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);
            }
            break;
            default:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                *viwers_container << std::move(image_display);
            }
            break;
            }
            minipage->construct(std::move(viwers_container), SK_ColorBLACK);
        }
        else
        {
            auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);

            switch (current_panel_arragement)
            {
            case Panels::ONE_PANEL:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                *viwers_container << std::move(image_display);
            }
            break;
            case Panels::TWO_PANELS:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Y);
                *viwers_container << std::move(image_display_x) << std::move(image_display_y);
            }
            break;
            case Panels::THREE_PANELS:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Y);
                std::unique_ptr<curan::ui::SlidingPanel> image_display_z = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::Z);
                *viwers_container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);
            }
            break;
            default:
            {
                std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);
                *viwers_container << std::move(image_display);
            }
            break;
            }
            auto text_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
            auto button_ac = MutatingTextPanel::make(true, "define ac point: e.g. p4");
            button_ac->set_background_color({1.0f, 1.0f, 1.0f, 1.0f}).set_text_color(SkColors::kBlack).set_default_text_color({.5f, .5f, .5f, 1.0f}).set_highlighted_color({.8f, .8f, .8f, 1.0f}).set_cursor_color({1.0, 0.0, 0.0, 1.0}).set_size(SkRect::MakeWH(200, 100));
            button_ac->add_textdefined_callback([&](MutatingTextPanel *button, const std::string &str, ConfigDraw *config)
                                                {
				int result{};
        		auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
       			if (ec == std::errc()){ //provided number is proper
                    size_t index = 0;
                    bool stroke_found = false;
                    map[current_volume].for_each(Direction::Z,[&](Mask& mask){
                        auto stroke = mask.find(result);
                        if(stroke){
                            stroke_found = true;
						    std::visit(curan::utilities::overloaded{
                                [&](const Path &path){
                                    
							    },						 
							    [&](const Point &point) { 
                                    ImageType::IndexType local_index;
                                    local_index[0] = (int) std::round(point.normalized_point.fX*map[current_volume].dimension(Direction::X)-1);
                                    local_index[1] = (int) std::round(point.normalized_point.fY*map[current_volume].dimension(Direction::Y)-1);
                                    local_index[2] = index;
                                    std::cout << "index ac is: " << local_index << std::endl;
                                    ImageType::PointType ac_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,ac_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lac_point = Eigen::Matrix<double,3,1>::Zero();
                                    lac_point[0] = ac_point_in_world_coordinates[0];
                                    lac_point[1] = ac_point_in_world_coordinates[1];
                                    lac_point[2] = ac_point_in_world_coordinates[2];
                                    ac_point = lac_point;
                                    if(config->stack_page!=nullptr){
						                config->stack_page->stack(create_overlay_with_success("added AC point"));
					                }
						        }},*stroke);
                        }
                        ++index;
                    });
                    if(!stroke_found)
                        if(config->stack_page!=nullptr){
						    config->stack_page->stack(create_overlay_with_warning("failed to find point, please insert new point"));
					    }
				}
        		else if (ec == std::errc::invalid_argument || ec == std::errc::result_out_of_range){
					if(config->stack_page!=nullptr){
						config->stack_page->stack(create_overlay_with_warning("failed to parse identifier"));
					}
				} });
            auto button_cp = MutatingTextPanel::make(true, "define pc point: e.g. p6");
            button_cp->set_background_color({1.0f, 1.0f, 1.0f, 1.0f}).set_text_color(SkColors::kBlack).set_default_text_color({.5f, .5f, .5f, 1.0f}).set_highlighted_color({.8f, .8f, .8f, 1.0f}).set_cursor_color({1.0, 0.0, 0.0, 1.0}).set_size(SkRect::MakeWH(200, 100));
            button_cp->add_textdefined_callback([&](MutatingTextPanel *button, const std::string &str, ConfigDraw *config)
                                                {
				int result{};
        		auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
       			if (ec == std::errc()){ //provided number is proper
                    size_t index = 0;
                    bool stroke_found = false;
                    map[current_volume].for_each(Direction::Z,[&](Mask& mask){
                        auto stroke = mask.find(result);
                        if(stroke){
                            stroke_found = true;
						    std::visit(curan::utilities::overloaded{
                                [&](const Path &path){
							
							    },						 
							    [&](const Point &point) { 
                                    ImageType::IndexType local_index;
                                    local_index[0] = (int) std::round(point.normalized_point.fX*map[current_volume].dimension(Direction::X)-1);
                                    local_index[1] = (int) std::round(point.normalized_point.fY*map[current_volume].dimension(Direction::Y)-1);
                                    local_index[2] = index;
                                    std::cout << "index cp is: " << local_index << std::endl;
                                    ImageType::PointType pc_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,pc_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lpc_point = Eigen::Matrix<double,3,1>::Zero();
                                    lpc_point[0] = pc_point_in_world_coordinates[0];
                                    lpc_point[1] = pc_point_in_world_coordinates[1];
                                    lpc_point[2] = pc_point_in_world_coordinates[2];
                                    pc_point = lpc_point;
                                    if(config->stack_page!=nullptr){
						                config->stack_page->stack(create_overlay_with_success("added CP point"));
					                }
						        }},*stroke);
                        }
                        ++index;
                    });
                    if(!stroke_found)
                        if(config->stack_page!=nullptr){
						    config->stack_page->stack(create_overlay_with_warning("failed to find point, please insert new point"));
					    }
				}
        		else if (ec == std::errc::invalid_argument || ec == std::errc::result_out_of_range){	
					if(config->stack_page!=nullptr){
						config->stack_page->stack(create_overlay_with_warning("failed to parse identifier"));
					}
				} });

            auto button_midpoint = MutatingTextPanel::make(true, "define midpoint: e.g. p10");
            button_midpoint->set_background_color({1.0f, 1.0f, 1.0f, 1.0f}).set_text_color(SkColors::kBlack).set_default_text_color({.5f, .5f, .5f, 1.0f}).set_highlighted_color({.8f, .8f, .8f, 1.0f}).set_cursor_color({1.0, 0.0, 0.0, 1.0}).set_size(SkRect::MakeWH(200, 100));
            button_midpoint->add_textdefined_callback([&](MutatingTextPanel *button, const std::string &str, ConfigDraw *config)
                                                      {
				int result{};
        		auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
       			if (ec == std::errc()){ //provided number is proper
                    size_t index = 0;
                    bool stroke_found = false;
                    map[current_volume].for_each(Direction::Z,[&](Mask& mask){
                        auto stroke = mask.find(result);
                        if(stroke){
                            stroke_found = true;
						    std::visit(curan::utilities::overloaded{
                                [&](const Path &path){
							
							    },						 
							    [&](const Point &point) { 
                                    ImageType::IndexType local_index{0,0,0};
                                    local_index[0] = (int) std::round(point.normalized_point.fX*map[current_volume].dimension(Direction::X)-1);
                                    local_index[1] = (int) std::round(point.normalized_point.fY*map[current_volume].dimension(Direction::Y)-1);
                                    local_index[2] = index;
                                    std::cout << "index midpoint is: " << local_index << std::endl;
                                    ImageType::PointType midpoint_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,midpoint_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lmid_point = Eigen::Matrix<double,3,1>::Zero();
                                    lmid_point[0] = midpoint_point_in_world_coordinates[0];
                                    lmid_point[1] = midpoint_point_in_world_coordinates[1];
                                    lmid_point[2] = midpoint_point_in_world_coordinates[2];
                                    midline = lmid_point;
                                    if(config->stack_page!=nullptr){
						                config->stack_page->stack(create_overlay_with_success("added midpoint"));
					                }
						        }},*stroke);
                        }
                        ++index;
                    });
                    if(!stroke_found)
                        if(config->stack_page!=nullptr){
						    config->stack_page->stack(create_overlay_with_warning("failed to find point, please insert new point"));
					    }
				}
        		else if (ec == std::errc::invalid_argument || ec == std::errc::result_out_of_range){
					if(config->stack_page!=nullptr){
						config->stack_page->stack(create_overlay_with_warning("failed to parse identifier"));
				    }
				} });

            auto perform_resampling = Button::make("Resample to AC-PC", resources);
            perform_resampling->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
            perform_resampling->add_press_call(
                [this](Button *button, Press press, ConfigDraw *config)
                {
                    if (!midline || !ac_point || !pc_point)
                    {
                        if (config->stack_page != nullptr)
                        {
                            config->stack_page->stack(create_overlay_with_warning("you must specify all points to resample the volume"));
                        }
                        midline = std::nullopt;
                        ac_point = std::nullopt;
                        pc_point = std::nullopt;
                        return;
                    }

                    Eigen::Matrix<double, 3, 1> orient_along_ac_pc = *pc_point - *ac_point;
                    if (orient_along_ac_pc.norm() < 1e-7)
                    {
                        if (config->stack_page != nullptr)
                        {
                            config->stack_page->stack(create_overlay_with_warning("vector is close to singular, try different pointss"));
                        }
                        midline = std::nullopt;
                        ac_point = std::nullopt;
                        pc_point = std::nullopt;
                        return;
                    }
                    orient_along_ac_pc.normalize();
                    Eigen::Matrix<double, 3, 1> orient_along_ac_midpoint = *midline - *ac_point;
                    if (orient_along_ac_midpoint.norm() < 1e-7)
                    {
                        if (config->stack_page != nullptr)
                        {
                            config->stack_page->stack(create_overlay_with_warning("vector is close to singular, try different pointss"));
                        }
                        midline = std::nullopt;
                        ac_point = std::nullopt;
                        pc_point = std::nullopt;
                        return;
                    }
                    orient_along_ac_midpoint.normalize();
                    Eigen::Matrix<double, 3, 1> orient_perpendic_to_ac_pc_ac_midline = orient_along_ac_pc.cross(orient_along_ac_midpoint);
                    if (orient_perpendic_to_ac_pc_ac_midline.norm() < 1e-7)
                    {
                        if (config->stack_page != nullptr)
                        {
                            config->stack_page->stack(create_overlay_with_warning("vector is close to singular, try different pointss"));
                        }
                        midline = std::nullopt;
                        ac_point = std::nullopt;
                        pc_point = std::nullopt;
                        return;
                    }

                    orient_perpendic_to_ac_pc_ac_midline.normalize();

                    orient_along_ac_midpoint = orient_perpendic_to_ac_pc_ac_midline.cross(orient_along_ac_pc);

                    orient_along_ac_midpoint.normalize();

                    auto input = map[PanelType::ORIGINAL_VOLUME].get_volume();

                    itk::Matrix<double, 3, 3> rotation_matrix;
                    rotation_matrix(0, 0) = orient_perpendic_to_ac_pc_ac_midline[0];
                    rotation_matrix(1, 0) = orient_perpendic_to_ac_pc_ac_midline[1];
                    rotation_matrix(2, 0) = orient_perpendic_to_ac_pc_ac_midline[2];

                    rotation_matrix(0, 1) = orient_along_ac_pc[0]; 
                    rotation_matrix(1, 1) = orient_along_ac_pc[1];
                    rotation_matrix(2, 1) = orient_along_ac_pc[2];

                    rotation_matrix(0, 2) = orient_along_ac_midpoint[0];
                    rotation_matrix(1, 2) = orient_along_ac_midpoint[1];
                    rotation_matrix(2, 2) = orient_along_ac_midpoint[2];

                    Eigen::Matrix<double,3,3> eigen_rotation_matrix;
                    for(size_t col = 0; col < 3 ; ++col)
                        for(size_t row = 0; row < 3 ; ++row)
                            eigen_rotation_matrix(row,col) = rotation_matrix(row,col);

                    Eigen::Matrix<double,3,1> origin_for_bounding_box{{input->GetOrigin()[0],input->GetOrigin()[1],input->GetOrigin()[2]}};
                    ImageType::PointType itk_along_dimension_x;
                    ImageType::IndexType index_along_x{(long long)input->GetLargestPossibleRegion().GetSize()[0],0,0};
                    input->TransformIndexToPhysicalPoint(index_along_x,itk_along_dimension_x);
                    Eigen::Matrix<double,3,1> extrema_along_x_for_bounding_box{{itk_along_dimension_x[0],itk_along_dimension_x[1],itk_along_dimension_x[2]}};
                    ImageType::PointType itk_along_dimension_y;
                    ImageType::IndexType index_along_y{0,(long long)input->GetLargestPossibleRegion().GetSize()[1],0};
                    input->TransformIndexToPhysicalPoint(index_along_y,itk_along_dimension_y);
                    Eigen::Matrix<double,3,1> extrema_along_y_for_bounding_box{{itk_along_dimension_y[0],itk_along_dimension_y[1],itk_along_dimension_y[2]}};
                    ImageType::PointType itk_along_dimension_z;
                    ImageType::IndexType index_along_z{0,0,(long long)input->GetLargestPossibleRegion().GetSize()[2]};
                    input->TransformIndexToPhysicalPoint(index_along_z,itk_along_dimension_z);
                    Eigen::Matrix<double,3,1> extrema_along_z_for_bounding_box{{itk_along_dimension_z[0],itk_along_dimension_z[1],itk_along_dimension_z[2]}};
                    Eigen::Matrix<double,3,1> spacing{{input->GetSpacing()[0],input->GetSpacing()[1],input->GetSpacing()[2]}};

                    BoundingBox bounding_box_original_image{origin_for_bounding_box,extrema_along_x_for_bounding_box,extrema_along_y_for_bounding_box,extrema_along_z_for_bounding_box,spacing};
                    auto output_bounding_box = bounding_box_original_image.centered_bounding_box<Strategy::CONSERVATIVE,false>(eigen_rotation_matrix);
                    using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
                    auto filter = FilterType::New();

                    using TransformType = itk::IdentityTransform<double, 3>;
                    auto transform = TransformType::New();

                    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
                    auto interpolator = InterpolatorType::New();
                    filter->SetInterpolator(interpolator);
                    filter->SetDefaultPixelValue(100);
                    filter->SetTransform(transform);

                    filter->SetInput(input);
                    filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0],output_bounding_box.origin[1],output_bounding_box.origin[2]}});
                    filter->SetOutputSpacing(ImageType::SpacingType{{output_bounding_box.spacing[0],output_bounding_box.spacing[1],output_bounding_box.spacing[2]}});
                    filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0],(size_t)output_bounding_box.size[1],(size_t)output_bounding_box.size[2]}});

                    for(size_t col = 0; col < 3 ; ++col)
                        for(size_t row = 0; row < 3 ; ++row)
                            rotation_matrix(row,col) = output_bounding_box.orientation(row,col);

                    filter->SetOutputDirection(rotation_matrix);

                    try{
                        filter->Update();
                        auto output = filter->GetOutput();
                        map[PanelType::RESAMPLED_VOLUME].update_volume(output);
                    } catch(...){
                        if (config->stack_page != nullptr)
                            config->stack_page->stack(create_overlay_with_warning("failed to resample volume to AC-PC"));
                    }

                    midline = std::nullopt;
                    ac_point = std::nullopt;
                    pc_point = std::nullopt;
                });
            *text_container << std::move(button_ac) << std::move(button_cp) << std::move(button_midpoint) << std::move(perform_resampling);
            auto total_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
            *total_container << std::move(text_container) << std::move(viwers_container);
            total_container->set_divisions({0.0, 0.1, 1.0});
            minipage->construct(std::move(total_container), SK_ColorBLACK);
        }
    }

    std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page(){
        using namespace curan::ui;
		using PixelType = unsigned char;
		auto item_explorer = ItemExplorer::make("file_icon.png",resources);
		using ImageType = itk::Image<PixelType, 3>;
		using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;
        size_t identifier = 0;
        for(auto& vol : map){
            if(vol.filled()){
                auto itk_pointer = vol.get_volume();
		        auto extract_filter = ExtractFilterType::New();
		        extract_filter->SetDirectionCollapseToSubmatrix();
		        extract_filter->SetInput(itk_pointer);

		        ImageType::RegionType inputRegion = itk_pointer->GetBufferedRegion();
		        ImageType::SpacingType spacing = itk_pointer->GetSpacing();
		        ImageType::SizeType size = inputRegion.GetSize();

		        auto copy_size = size;
		        size[Direction::Z] = 1;

		        ImageType::IndexType start = inputRegion.GetIndex();
		        start[Direction::Z] = std::floor(copy_size[Direction::Z]/2.0);
		        ImageType::RegionType desiredRegion;
		        desiredRegion.SetSize(size);
		        desiredRegion.SetIndex(start);
		        extract_filter->SetExtractionRegion(desiredRegion);
		        extract_filter->UpdateLargestPossibleRegion();

		        ImageType::Pointer pointer_to_block_of_memory = extract_filter->GetOutput();
                SkImageInfo information = SkImageInfo::Make(size[Direction::X], size[Direction::Y], kGray_8_SkColorType, kOpaque_SkAlphaType);
	            auto pixmap = SkPixmap(information, pointer_to_block_of_memory->GetBufferPointer(), size[Direction::Y] * sizeof(PixelType));
                item_explorer->add(Item{identifier,SkSurfaces::WrapPixels(pixmap)->makeImageSnapshot(),"volume_"+std::to_string(identifier)});
	            
            }
            ++identifier;
        }

        item_explorer->set_size(SkRect::MakeWH(800,400));

        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*container << std::move(item_explorer);


        return Overlay::make(std::move(container), SkColorSetARGB(10, 125, 125, 125), true);
    }


    std::unique_ptr<curan::ui::Overlay> create_layout_page()
    {
        using namespace curan::ui;
        auto button = Button::make(" ", "layout1x1.png", resources);
        button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
        button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                               { current_panel_arragement = Panels::ONE_PANEL; create_panel_ac_pc_instructions(); });

        auto button2 = Button::make(" ", "layout1x2.png", resources);
        button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
        button2->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                { current_panel_arragement = Panels::TWO_PANELS;create_panel_ac_pc_instructions(); });

        auto button3 = Button::make(" ", "layout1x3.png", resources);
        button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
        button3->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                { current_panel_arragement = Panels::THREE_PANELS;create_panel_ac_pc_instructions(); });

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
        *viwers_container << std::move(button) << std::move(button2) << std::move(button3);
        viwers_container->set_color(SK_ColorTRANSPARENT);

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Overlay> create_option_page()
    {
        using namespace curan::ui;
        auto button = Button::make("Layout", resources);
        button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                               {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(create_layout_page());
		} });

        auto button2 = Button::make("Resample AC-PC", resources);
        button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((is_acpc_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button2->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                {
			is_acpc_being_defined = !is_acpc_being_defined;
			create_panel_ac_pc_instructions(); });

        auto button3 = Button::make("Define Trajectory", resources);
        button3->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button3->add_press_call([&](Button *button, Press press, ConfigDraw *config) {

        });

        auto button4 = Button::make("Change Volume", resources);
        button4->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button4->add_press_call([&](Button *button, Press press, ConfigDraw *config) {
            if(config->stack_page!=nullptr){
			    config->stack_page->stack(create_volume_explorer_page());
		    }
        });

        auto button5 = Button::make("Load Series", resources);
        button5->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button5->add_press_call([&](Button *button, Press press, ConfigDraw *config) {

        });    

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
        *viwers_container << std::move(button) << std::move(button2) << std::move(button3) << std::move(button4) << std::move(button5);
        viwers_container->set_color(SK_ColorTRANSPARENT);

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Container> generate_main_page_content()
    {
        std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &map[current_volume], curan::ui::Direction::X);

        auto container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
        *container << std::move(image_display);

        std::unique_ptr<curan::ui::MiniPage> lminipage = curan::ui::MiniPage::make(std::move(container), SK_ColorBLACK);
        minipage = lminipage.get();
        lminipage->add_key_call([this](curan::ui::MiniPage *minipage, curan::ui::Key arg, curan::ui::ConfigDraw *draw)
                                {
			if (arg.key == GLFW_KEY_H && arg.action == GLFW_PRESS){
				if(draw->stack_page!=nullptr){
					draw->stack_page->stack(create_option_page());
				}
			} });

        auto minimage_container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
        *minimage_container << std::move(lminipage);
        return minimage_container;
    }
};

std::optional<ImageType::Pointer> get_volume(std::string path)
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

    auto seriesItr = seriesUID.begin();
    auto seriesEnd = seriesUID.end();
    while (seriesItr != seriesEnd)
    {
        std::cout << seriesItr->c_str() << std::endl;
        ++seriesItr;
    }

    std::string seriesIdentifier;
    seriesIdentifier = seriesUID.begin()->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(reader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

    using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }

    return filter->GetOutput();
}

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 2200, 1200};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524");
        if (!volume)
            return 1;

        DataSpecificApplication data_application{*volume, resources};

        curan::ui::Page page{std::move(data_application.generate_main_page_content()), SK_ColorBLACK};

        ConfigDraw config{&page};

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();
            if (viewer->was_updated())
            {
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
    catch (const std::exception &e)
    {
        std::cout << "Exception thrown:" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
}