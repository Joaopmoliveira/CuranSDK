#include "UserInterface.h"
#include "BoundingBox.h"
#include "utils/Overloading.h"
#include "utils/Overloading.h"
#include "geometry/Polyheadra.h"

uint_least8_t dicom_compliant_conversion[256] = {0, 30, 33, 35, 37, 39, 40, 41, 43, 44, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 54, 55, 55, 56, 57, 57, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 65, 66, 67, 67, 68, 69, 69, 70, 71, 71, 72, 73, 73, 74, 75, 76, 76, 77, 77, 78, 79, 80, 80, 80, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 88, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 98, 98, 99, 100, 101, 101, 102, 103, 103, 104, 105, 106, 106, 107, 108, 109, 109, 110, 111, 111, 112, 113, 113, 114, 115, 116, 116, 117, 118, 119, 119, 120, 121, 122, 123, 123, 124, 125, 126, 126, 127, 128, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 136, 137, 138, 139, 140, 141, 141, 142, 143, 144, 145, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 158, 158, 159, 160, 161, 162, 163, 164, 164, 166, 167, 167, 169, 170, 170, 171, 172, 173, 174, 175, 176, 176, 178, 178, 180, 181, 182, 183, 184, 184, 186, 186, 188, 188, 189, 191, 191, 192, 194, 194, 196, 197, 197, 199, 200, 201, 202, 203, 204, 205, 207, 207, 208, 209, 210, 212, 213, 214, 215, 216, 217, 218, 220, 221, 222, 223, 224, 225, 226, 228, 229, 230, 231, 233, 234, 235, 236, 238, 239, 240, 241, 242, 244, 245, 246, 248, 249, 250, 251, 253, 254, 255};
constexpr bool use_dicom_compliance = false;

void Application::compute_point(curan::ui::VolumetricMask *vol_mas, const curan::ui::directed_stroke &dir_stroke, curan::ui::ConfigDraw *config)
{

    // now we need to convert between itk coordinates and real world coordinates
    if (!(dir_stroke.point_in_image_coordinates.cols() > 0))
        throw std::runtime_error("the collumns of the highlighted path must be at least 1");

    if (ptr_config && ptr_config->stack_page && dir_stroke.point_in_image_coordinates.cols() > 1 && !is_roi_being_specified)
    {
        ptr_config->stack_page->stack(warning_overlay("must select a single point, not a path"));
        return;
    }

    ImageType::IndexType local_index;
    ImageType::PointType itk_point_in_world_coordinates;
    Eigen::Matrix<double, 3, 2> min_max_coefficients = Eigen::Matrix<double, 3, 2>::Zero();
    if (is_roi_being_specified)
    {
        min_max_coefficients.col(0) = dir_stroke.point_in_image_coordinates.rowwise().minCoeff();
        min_max_coefficients.col(1) = dir_stroke.point_in_image_coordinates.rowwise().maxCoeff();
    }
    else
    {
        local_index[0] = dir_stroke.point_in_image_coordinates(0, 0);
        local_index[1] = dir_stroke.point_in_image_coordinates(1, 0);
        local_index[2] = dir_stroke.point_in_image_coordinates(2, 0);
        vol_mas->get_volume()->TransformIndexToPhysicalPoint(local_index, itk_point_in_world_coordinates);

        min_max_coefficients(0, 0) = itk_point_in_world_coordinates[0];
        min_max_coefficients(1, 0) = itk_point_in_world_coordinates[1];
        min_max_coefficients(2, 0) = itk_point_in_world_coordinates[2];
    }

    if (is_first_point_being_defined)
    {
        if (is_roi_being_specified)
            first_path = min_max_coefficients;
        else
            first_point = min_max_coefficients.col(0);
        if (ptr_button_ac_point)
            ptr_button_ac_point->set_click_color(SK_ColorGRAY)
                .set_hover_color(SK_ColorCYAN)
                .set_waiting_color(SK_ColorLTGRAY)
                .set_size(SkRect::MakeWH(200, 200));
    }
    else if (is_second_point_being_defined)
    {
        if (is_roi_being_specified)
            second_path = min_max_coefficients;
        else
            second_point = min_max_coefficients.col(0);
        if (ptr_button_pc_point)
            ptr_button_pc_point->set_click_color(SK_ColorGRAY)
                .set_hover_color(SK_ColorCYAN)
                .set_waiting_color(SK_ColorLTGRAY)
                .set_size(SkRect::MakeWH(200, 200));
    }
    else if (is_third_point_being_defined)
    {
        third_point = min_max_coefficients.col(0);
        if (ptr_button_midpoint)
            ptr_button_midpoint->set_click_color(SK_ColorGRAY)
                .set_hover_color(SK_ColorCYAN)
                .set_waiting_color(SK_ColorLTGRAY)
                .set_size(SkRect::MakeWH(200, 200));
    }
};

Application::Application(curan::ui::IconResources &in_resources, std::string_view path_to_load, std::mutex &inmut) : resources{in_resources},
                                                                                                                map{{{nullptr}}},
                                                                                                                path{path_to_load},
                                                                                                                mut{inmut}
{
    using namespace curan::ui;
    map[PanelType::ORIGINAL_VOLUME].add_pressedhighlighted_call(
        [this](VolumetricMask *vol_mas, ConfigDraw *config_draw, const directed_stroke &strokes)
        {
            compute_point(vol_mas, strokes, config_draw);
        });
    using ImageReaderType = itk::ImageFileReader<ImageType>;

    std::printf("\nReading input volumes...\n");
    auto fixedImageReader = ImageReaderType::New();

    fixedImageReader->SetFileName(path_to_load.data());

    try{
        fixedImageReader->Update();
         map[PanelType::ORIGINAL_VOLUME].update_volume(fixedImageReader->GetOutput());
    } catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return;
    }
}

std::unique_ptr<curan::ui::Overlay> Application::warning_overlay(const std::string &warning)
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

std::unique_ptr<curan::ui::Overlay> Application::success_overlay(const std::string &success)
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

void Application::view_image_simple()
{
    using namespace curan::ui;
    auto button = Button::make("Layout", resources);
    button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                           {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay());
		} });

    std::unique_ptr<Button> button6 = Button::make("Registration ROI", resources);
    button6->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button6->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                            {
        are_points_being_defined = !are_points_being_defined;
        is_roi_being_specified = !is_roi_being_specified;
		point_selection(); });


    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(button) << std::move(button6);
    viwers_container->set_color(SK_ColorTRANSPARENT);

    auto image_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);

    switch (current_panel_arragement)
    {
    case Panels::ONE_PANEL:
    {
        std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        if (use_dicom_compliance)
            image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        *image_container << std::move(image_display);
    }
    break;
    case Panels::TWO_PANELS:
    {
        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &map[current_volume], Direction::Y);
        if (use_dicom_compliance)
        {
            image_display_x->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_y->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }

        *image_container << std::move(image_display_x) << std::move(image_display_y);
    }
    break;
    case Panels::THREE_PANELS:
    {
        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &map[current_volume], Direction::Y);
        std::unique_ptr<SlidingPanel> image_display_z = SlidingPanel::make(resources, &map[current_volume], Direction::Z);
        if (use_dicom_compliance)
        {
            image_display_x->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_y->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_z->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);
    }
    break;
    default:
    {
        std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        if (use_dicom_compliance)
        {
            image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display);
    }
    break;
    }

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_container);
    container->set_divisions({0.0, 0.1, 1.0});

    minipage->construct(std::move(container), SK_ColorBLACK);
}

void Application::view_roi_selection()
{
    using namespace curan::ui;
    auto button = Button::make("Layout", resources);
    button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                           {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay());
		} });

    std::unique_ptr<Button> button2 = Button::make("Registration ROI", resources);
    button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((are_points_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button2->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                            {
            is_roi_being_specified = !is_roi_being_specified;
            are_points_being_defined = !are_points_being_defined;
		    point_selection(); });
    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(button) << std::move(button2);

    viwers_container->set_color(SK_ColorTRANSPARENT);

    auto image_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);

    switch (current_panel_arragement)
    {
    case Panels::ONE_PANEL:
    {
        std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        if (use_dicom_compliance)
        {
            image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display);
    }
    break;
    case Panels::TWO_PANELS:
    {
        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &map[current_volume], Direction::Y);
        if (use_dicom_compliance)
        {
            image_display_x->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_y->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display_x) << std::move(image_display_y);
    }
    break;
    case Panels::THREE_PANELS:
    {
        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &map[current_volume], Direction::Y);
        std::unique_ptr<SlidingPanel> image_display_z = SlidingPanel::make(resources, &map[current_volume], Direction::Z);
        if (use_dicom_compliance)
        {
            image_display_x->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_y->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
            image_display_z->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);
    }
    break;
    default:
    {
        std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources, &map[current_volume], Direction::X);
        if (use_dicom_compliance)
        {
            image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
        }
        *image_container << std::move(image_display);
    }
    break;
    }
    auto text_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    auto button_ac_point = Button::make("path 1", "", resources);
    button_ac_point->set_click_color(SK_ColorGRAY)
        .set_hover_color(SK_ColorLTGRAY)
        .set_waiting_color(SK_ColorDKGRAY)
        .set_size(SkRect::MakeWH(200, 200));
    auto button_pc_point = Button::make("path 2", "", resources);
    button_pc_point->set_click_color(SK_ColorGRAY)
        .set_hover_color(SK_ColorLTGRAY)
        .set_waiting_color(SK_ColorDKGRAY)
        .set_size(SkRect::MakeWH(200, 200));

    ptr_button_ac_point = button_ac_point.get();
    ptr_button_pc_point = button_pc_point.get();

    button_ac_point->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                    { 
            is_first_point_being_defined=!is_first_point_being_defined;
            if(is_first_point_being_defined){
                    button->set_waiting_color(SK_ColorGRAY).set_click_color(SK_ColorCYAN); 
                    is_second_point_being_defined =false;
                    is_third_point_being_defined =false;
                    if(ptr_button_pc_point && !second_point) 
                        ptr_button_pc_point->set_waiting_color(SK_ColorDKGRAY)
                            .set_click_color(SK_ColorGRAY);
                    if(ptr_button_midpoint && !third_point) 
                        ptr_button_midpoint->set_waiting_color(SK_ColorDKGRAY)
                            .set_click_color(SK_ColorGRAY);
            } else {
                button->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorLTGRAY)
                    .set_waiting_color(SK_ColorDKGRAY);
            } });

    button_pc_point->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                    { 
            is_second_point_being_defined=!is_second_point_being_defined;
            if(is_second_point_being_defined){
                button->set_waiting_color(SK_ColorGRAY).set_click_color(SK_ColorCYAN); 
                is_first_point_being_defined =false;
                is_third_point_being_defined =false;
                if(ptr_button_ac_point && !first_point) 
                    ptr_button_ac_point->set_waiting_color(SK_ColorDKGRAY)
                        .set_click_color(SK_ColorGRAY);
                if(ptr_button_midpoint && !third_point) 
                    ptr_button_midpoint->set_waiting_color(SK_ColorDKGRAY)
                        .set_click_color(SK_ColorGRAY);
            } else {
                button->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorLTGRAY)
                    .set_waiting_color(SK_ColorDKGRAY);
            } });

    auto create_geometry = Button::make("Push Mask", "", resources);
    create_geometry->set_hover_color(SK_ColorDKGRAY)
        .set_waiting_color(SK_ColorBLACK)
        .set_size(SkRect::MakeWH(200, 80));
    create_geometry->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                    {
            if (config->stack_page != nullptr) config->stack_page->stack(success_overlay("generating geometry..."));
            curan::utilities::Job job{"resampling volume", [this, config]()
            {
                if (!first_path || !second_path)
                {
                    std::string s = !first_path ? "1 " : "2 ";
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("points :"+s+" problematic"));
                    clear_all_paths_and_points();
                    return;
                }

                Eigen::Matrix<double,3,4> min_and_max_both_paths = Eigen::Matrix<double,3,4>::Zero();
                min_and_max_both_paths.col(0) = (*first_path).col(0);
                min_and_max_both_paths.col(1) = (*first_path).col(1);
                min_and_max_both_paths.col(2) = (*second_path).col(0);
                min_and_max_both_paths.col(3) = (*second_path).col(1);

                Eigen::Matrix<double,3,1> min_coeff_all = min_and_max_both_paths.rowwise().minCoeff();
                Eigen::Matrix<double,3,1> max_coeff_all = min_and_max_both_paths.rowwise().maxCoeff();

                min_coeff_all[0] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[0];
                min_coeff_all[1] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[1];
                min_coeff_all[2] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[2];
            
                max_coeff_all[0] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[0];
                max_coeff_all[1] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[1];
                max_coeff_all[2] /= map[current_volume].get_volume()->GetLargestPossibleRegion().GetSize()[2];

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

                rotation_and_scalling_matrix = Eigen::Matrix<double,4,4>::Identity();
                rotation_and_scalling_matrix.block<3,1>(0,3) = origin;

                map[current_volume].add_geometry(geom);

                if (config->stack_page != nullptr) {
                    config->stack_page->replace_last(success_overlay("generated geometry!"));
                    are_points_being_defined = !are_points_being_defined;
                    is_roi_being_specified = !is_roi_being_specified;
                    point_selection();
                }
                clear_all_paths_and_points();
            }};
            pool->submit(job); });

    *text_container << std::move(button_ac_point) << std::move(button_pc_point) << std::move(create_geometry);

    auto image_and_text_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *image_and_text_container << std::move(text_container) << std::move(image_container);
    image_and_text_container->set_divisions({0.0, 0.1, 1.0});

    auto total_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *total_container << std::move(viwers_container) << std::move(image_and_text_container);
    total_container->set_divisions({0.0, 0.1, 1.0});
    minipage->construct(std::move(total_container), SK_ColorBLACK);
}

void Application::point_selection()
{
    ptr_button_ac_point = nullptr;
    ptr_button_pc_point = nullptr;
    ptr_button_midpoint = nullptr;

    if (are_points_being_defined && is_roi_being_specified)
        view_roi_selection();
    else
        view_image_simple();
}

std::unique_ptr<curan::ui::Overlay> Application::layout_overlay()
{
    using namespace curan::ui;

    auto button = Button::make(" ", "layout1x1.png", resources);
    button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                               { current_panel_arragement = Panels::ONE_PANEL; point_selection(); });

    auto button2 = Button::make(" ", "layout1x2.png", resources);
    button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    button2->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                { current_panel_arragement = Panels::TWO_PANELS;point_selection(); });

    auto button3 = Button::make(" ", "layout1x3.png", resources);
    button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
    button3->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                { current_panel_arragement = Panels::THREE_PANELS;point_selection(); });

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(button) << std::move(button2) << std::move(button3);
    viwers_container->set_color(SK_ColorTRANSPARENT);
    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

std::unique_ptr<curan::ui::Container> Application::main_page()
{
    using namespace curan::ui;
    std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources, &map[current_volume], Direction::X);
    if (use_dicom_compliance)
    {
        image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
    }
        using namespace curan::ui;
    auto button = Button::make("Layout", resources);
    button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                           {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay());
		} });

    std::unique_ptr<Button> button2 = Button::make("Registration ROI", resources);
    button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((are_points_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
    button2->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                            {
            is_roi_being_specified = !is_roi_being_specified;
            are_points_being_defined = !are_points_being_defined;
		    point_selection(); });
    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
    *viwers_container << std::move(button) << std::move(button2);

    viwers_container->set_color(SK_ColorTRANSPARENT);

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *container << std::move(viwers_container) << std::move(image_display);
    container->set_divisions({0.0, 0.1, 1.0});

    std::unique_ptr<MiniPage> lminipage = MiniPage::make(std::move(container), SK_ColorBLACK);
    minipage = lminipage.get();
    auto minimage_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *minimage_container << std::move(lminipage);
    return minimage_container;
}
