#include "UserInterface.h"
#include "BoundingBox.h"
#include "utils/Overloading.h"

    void Application::compute_point(const curan::ui::directed_stroke& dir_stroke, curan::ui::ConfigDraw* config){
        std::optional<Eigen::Matrix<double, 3, 1>> possible_point;
        size_t index = 0;

        map[current_volume].for_each(curan::ui::Direction::Z,[&](curan::ui::Mask& mask){
			std::visit(curan::utilities::overloaded{
                [&](const curan::ui::Path &path){
                                    
			    },						 
			    [&](const curan::ui::Point &point) { 
                    ImageType::IndexType local_index;
                    local_index[0] = (int) std::round(point.normalized_point.fX*map[current_volume].dimension(curan::ui::Direction::X)-1);
                    local_index[1] = (int) std::round(point.normalized_point.fY*map[current_volume].dimension(curan::ui::Direction::Y)-1);
                    local_index[2] = index;
                    ImageType::PointType point_in_world_coordinates;
                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,point_in_world_coordinates);
                    Eigen::Matrix<double,3,1> local_point = Eigen::Matrix<double,3,1>::Zero();
                    local_point[0] = point_in_world_coordinates[0];
                    local_point[1] = point_in_world_coordinates[1];
                    local_point[2] = point_in_world_coordinates[2];
                    possible_point = local_point;
		    }
            },dir_stroke.stroke);
        ++index;
        });
        if(!possible_point && config->stack_page!=nullptr)
			config->stack_page->stack(warning_overlay("failed to find point, please insert new point"));
        if(is_first_point_being_defined){
            first_point = possible_point;
            if(ptr_button_ac_point) 
                ptr_button_ac_point->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorCYAN)
                    .set_waiting_color(SK_ColorLTGRAY)
                    .set_size(SkRect::MakeWH(200, 200));
        }
        else if(is_second_point_being_defined){
            second_point = possible_point;
            if(ptr_button_pc_point) 
                ptr_button_pc_point->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorCYAN)
                    .set_waiting_color(SK_ColorLTGRAY)
                    .set_size(SkRect::MakeWH(200, 200));
        }
        else if(is_third_point_being_defined){
            third_point = possible_point;
            if(ptr_button_midpoint) 
                ptr_button_midpoint->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorCYAN)
                    .set_waiting_color(SK_ColorLTGRAY)
                    .set_size(SkRect::MakeWH(200, 200));
        }
    };

    Application::Application(ImageType::Pointer volume, curan::ui::IconResources &in_resources) : 
        resources{in_resources}, 
        map{{{volume}, {nullptr}, {nullptr}}}
    {
        using namespace curan::ui;
        map[PanelType::ORIGINAL_VOLUME].add_pressedhighlighted_call(
            [this](VolumetricMask* vol_mas, ConfigDraw* config_draw , const std::vector<curan::ui::directed_stroke>& strokes){
            if(strokes.size()>0)
                compute_point(strokes.back(),config_draw);
        });
        map[PanelType::RESAMPLED_VOLUME].add_pressedhighlighted_call(
            [this](VolumetricMask* vol_mas, ConfigDraw* config_draw, const std::vector<curan::ui::directed_stroke>& strokes){
            if(strokes.size()>0)
                compute_point(strokes.back(),config_draw);
        });
        map[PanelType::TRAJECTORY_ORIENTED_VOLUME].add_pressedhighlighted_call(
            [this](VolumetricMask* vol_mas, ConfigDraw* config_draw , const std::vector<curan::ui::directed_stroke>& strokes){
            if(strokes.size()>0)
                compute_point(strokes.back(),config_draw);
        });
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

    void Application::view_image_simple(){
        using namespace curan::ui;
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

    void Application::view_image_with_point_selection(){
        using namespace curan::ui;
        
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
        auto button_ac_point = (current_volume == PanelType::ORIGINAL_VOLUME) ? 
                                    Button::make("ac point", "", resources) :
                                    Button::make("target", "", resources);
        button_ac_point->set_click_color(SK_ColorGRAY)
            .set_hover_color(SK_ColorLTGRAY)
            .set_waiting_color(SK_ColorDKGRAY)
            .set_size(SkRect::MakeWH(200, 200));
        auto button_pc_point = (current_volume == PanelType::ORIGINAL_VOLUME) ? 
                                    Button::make("pc point", "", resources) : 
                                    Button::make("plane xy", "", resources);
        button_pc_point->set_click_color(SK_ColorGRAY)
            .set_hover_color(SK_ColorLTGRAY)
            .set_waiting_color(SK_ColorDKGRAY)
            .set_size(SkRect::MakeWH(200, 200));
        auto button_midpoint = (current_volume == PanelType::ORIGINAL_VOLUME) ? 
                                    Button::make("mid point", "", resources) :  
                                    Button::make("entry", "", resources);
        button_midpoint->set_click_color(SK_ColorGRAY)
            .set_hover_color(SK_ColorLTGRAY)
            .set_waiting_color(SK_ColorDKGRAY)
            .set_size(SkRect::MakeWH(200, 200));

        ptr_button_ac_point = button_ac_point.get();
        ptr_button_pc_point = button_pc_point.get();
        ptr_button_midpoint = button_midpoint.get();

        
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
            } 
        });
        
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
            }
        });
        
        button_midpoint->add_press_call([this](Button *button, Press press, ConfigDraw *config)
        { 
            is_third_point_being_defined=!is_third_point_being_defined;
            if(is_third_point_being_defined){
                button->set_waiting_color(SK_ColorGRAY).set_click_color(SK_ColorCYAN); 
                is_first_point_being_defined =false;
                is_second_point_being_defined =false;
                if(ptr_button_ac_point  && !first_point) 
                    ptr_button_ac_point->set_waiting_color(SK_ColorDKGRAY)
                        .set_click_color(SK_ColorGRAY);
                if(ptr_button_pc_point  && !second_point) 
                    ptr_button_pc_point->set_waiting_color(SK_ColorDKGRAY)
                        .set_click_color(SK_ColorGRAY);
            } else {
                button->set_click_color(SK_ColorGRAY)
                    .set_hover_color(SK_ColorLTGRAY)
                    .set_waiting_color(SK_ColorDKGRAY);
            }
        });

        auto perform_resampling = (current_volume == PanelType::ORIGINAL_VOLUME) ? 
                                    Button::make("Resample to AC-PC", resources) :  
                                    Button::make("Trajectory resampling", "", resources);
        perform_resampling->set_hover_color(SK_ColorDKGRAY)
            .set_waiting_color(SK_ColorBLACK)
            .set_size(SkRect::MakeWH(200, 80));
        perform_resampling->add_press_call([this](Button *button, Press press, ConfigDraw *config)
        {
            if (config->stack_page != nullptr) config->stack_page->stack(success_overlay("resampling volume..."));
            curan::utilities::Job job{"resampling volume", [this, config]()
            {
                if (!first_point || !second_point || !third_point)
                {
                    std::string s = !first_point ? "1 " : (!second_point ? "2 " : ((!third_point) ? "3 " : " "));
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("points :"+s+" problematic"));
                    first_point = std::nullopt;
                    second_point = std::nullopt;
                    third_point = std::nullopt;
                    return;
                }

                Eigen::Matrix<double, 3, 1> orient_along_ac_pc = *second_point - *first_point;
                if (orient_along_ac_pc.norm() < 1e-7)
                {
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("singular (1-2) vector, try different points"));
                    first_point = std::nullopt;
                    second_point = std::nullopt;
                    third_point = std::nullopt;
                    return;
                }
                
                orient_along_ac_pc.normalize();
                Eigen::Matrix<double, 3, 1> orient_along_ac_midpoint = *third_point - *first_point;
                
                if (orient_along_ac_midpoint.norm() < 1e-7)
                {
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("singular (1-3) vector, try different points"));
                    first_point = std::nullopt;
                    second_point = std::nullopt;
                    third_point = std::nullopt;
                    return;
                }
                orient_along_ac_midpoint.normalize();
                Eigen::Matrix<double, 3, 1> orient_perpendic_to_ac_pc_ac_midline = orient_along_ac_pc.cross(orient_along_ac_midpoint);

                if (orient_perpendic_to_ac_pc_ac_midline.norm() < 1e-7)
                {
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("cross singular vector, try different points"));                         
                    first_point = std::nullopt;
                    second_point = std::nullopt;
                    third_point = std::nullopt;
                    return;
                }

                orient_perpendic_to_ac_pc_ac_midline.normalize();

                orient_along_ac_midpoint = orient_perpendic_to_ac_pc_ac_midline.cross(orient_along_ac_pc);

                orient_along_ac_midpoint.normalize();

                // we always use the original volume instead of the current volume,
                // or else the reconstructed volume will always increase in size
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

                Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
                Eigen::Matrix<double, 3, 3> original_eigen_rotation_matrix;
                auto direction = input->GetDirection();
                for (size_t col = 0; col < 3; ++col)
                    for (size_t row = 0; row < 3; ++row)
                    {
                        original_eigen_rotation_matrix(row, col) = direction(row, col);
                        eigen_rotation_matrix(row, col) = rotation_matrix(row, col);
                    }

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
                filter->SetDefaultPixelValue(100);
                filter->SetTransform(transform);

                filter->SetInput(input);
                filter->SetOutputOrigin(itk::Point<double>{{output_bounding_box.origin[0], output_bounding_box.origin[1], output_bounding_box.origin[2]}});
                filter->SetOutputSpacing(ImageType::SpacingType{{output_bounding_box.spacing[0], output_bounding_box.spacing[1], output_bounding_box.spacing[2]}});
                filter->SetSize(itk::Size<3>{{(size_t)output_bounding_box.size[0], (size_t)output_bounding_box.size[1], (size_t)output_bounding_box.size[2]}});

                for (size_t col = 0; col < 3; ++col)
                    for (size_t row = 0; row < 3; ++row)
                        rotation_matrix(row, col) = output_bounding_box.orientation(row, col);

                filter->SetOutputDirection(rotation_matrix);

                try{
                    filter->Update();
                    auto output = filter->GetOutput();
                    switch (current_volume)
                    {
                    case PanelType::ORIGINAL_VOLUME:
                        map[PanelType::RESAMPLED_VOLUME].update_volume(output);
                    break;
                    case PanelType::RESAMPLED_VOLUME:
                        map[PanelType::TRAJECTORY_ORIENTED_VOLUME].update_volume(output);
                    break;
                    case PanelType::TRAJECTORY_ORIENTED_VOLUME:
                        map[PanelType::TRAJECTORY_ORIENTED_VOLUME].update_volume(output);
                    break;
                    default:
                        if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("failure to process volume"));
                    break;
                    }
                    if (config->stack_page != nullptr) {
                        config->stack_page->replace_last(success_overlay("resampled volume!"));
                        is_acpc_being_defined = false;
                        point_selection();
                        if(current_volume==PanelType::RESAMPLED_VOLUME){
                            final_first_point = first_point;
                            final_second_point = second_point;
                            final_third_point = third_point;
                        }
                    }
                }
                catch (...){
                    if (config->stack_page != nullptr) config->stack_page->replace_last(warning_overlay("failed to resample volume to AC-PC"));
                }

                first_point = std::nullopt;
                second_point = std::nullopt;
                third_point = std::nullopt;
            }};
            pool->submit(job);
            });

            *text_container << std::move(button_ac_point) << std::move(button_pc_point) << std::move(button_midpoint) << std::move(perform_resampling);
            auto total_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
            *total_container << std::move(text_container) << std::move(viwers_container);
            total_container->set_divisions({0.0, 0.1, 1.0});
            minipage->construct(std::move(total_container), SK_ColorBLACK);
    }

    void Application::point_selection(){
        ptr_button_ac_point = nullptr;
        ptr_button_pc_point = nullptr;
        ptr_button_midpoint = nullptr;
        if(is_acpc_being_defined)
            view_image_with_point_selection();
        else
            view_image_simple();
    }

    std::unique_ptr<curan::ui::Overlay> Application::create_volume_explorer_page()
    {
        using namespace curan::ui;
        using PixelType = unsigned char;
        auto item_explorer = ItemExplorer::make("file_icon.png", resources);
        item_explorer->add_press_call([this](ItemExplorer *widget, Press press, ConfigDraw *draw)
                                      {
            auto highlighted = widget->highlighted();
            assert(highlighted.size()==1 && "the size is larger than one");
            switch(highlighted.front()){
                case PanelType::ORIGINAL_VOLUME:
                current_volume = PanelType::ORIGINAL_VOLUME;
                break;
                case PanelType::RESAMPLED_VOLUME:
                current_volume = PanelType::RESAMPLED_VOLUME;
                break;
                case PanelType::TRAJECTORY_ORIENTED_VOLUME:
                current_volume = PanelType::TRAJECTORY_ORIENTED_VOLUME;
                current_panel_arragement = Panels::ONE_PANEL;
                break;
                default:
                throw std::runtime_error("failed to select the proper index");
                break;
            }
            point_selection(); });
        using ImageType = itk::Image<PixelType, 3>;
        using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;
        size_t identifier = 0;
        for (auto &vol : map)
        {
            if (vol.filled())
            {
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
                item_explorer->add(Item{identifier, "vol_" + std::to_string(identifier), buff, extracted_size[0], extracted_size[1]});
            }
            ++identifier;
        }

        item_explorer->set_size(SkRect::MakeWH(800, 400));

        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        *container << std::move(item_explorer);

        return Overlay::make(std::move(container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Overlay> Application::layout_overlay()
    {
        using namespace curan::ui;
        if (current_volume == PanelType::TRAJECTORY_ORIENTED_VOLUME)
        {
            auto button = Button::make(" ", "layout1x1.png", resources);
            button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
            button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                   { current_panel_arragement = Panels::ONE_PANEL; point_selection(); });

            auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
            *viwers_container << std::move(button);
            viwers_container->set_color(SK_ColorTRANSPARENT);
            return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
        }
        else
        {
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
    }

    std::unique_ptr<curan::ui::Overlay> Application::option_overlay()
    {
        using namespace curan::ui;
        auto button = Button::make("Layout", resources);
        button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                               {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(layout_overlay());
		} });

        std::unique_ptr<curan::ui::Button> button2;

        switch (current_volume){
        case PanelType::ORIGINAL_VOLUME:
            button2 = Button::make("Resample AC-PC", resources);
            button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((is_acpc_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
            button2->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                    {
            if(current_volume==PanelType::TRAJECTORY_ORIENTED_VOLUME){
                if (config->stack_page != nullptr)
                    config->stack_page->stack(warning_overlay("cannot resample processed volume"));
                return;
            }
		    is_acpc_being_defined = !is_acpc_being_defined;
		    point_selection(); });
        case PanelType::RESAMPLED_VOLUME:
            button2 = Button::make("Define Trajectory", resources);
            button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((is_acpc_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
            button2->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                    {
            if(current_volume==PanelType::TRAJECTORY_ORIENTED_VOLUME){
                if (config->stack_page != nullptr)
                    config->stack_page->stack(warning_overlay("cannot resample processed volume"));
                return;
            }
		    is_acpc_being_defined = !is_acpc_being_defined;
		    point_selection(); });
        default:
            std::cout << "default\n";
            break;
        };

        auto button4 = Button::make("Change Volume", resources);
        button4->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button4->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                {
            if(config->stack_page!=nullptr){
			    config->stack_page->stack(create_volume_explorer_page());
		    } });

        auto button5 = Button::make("Load Series", resources);
        button5->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button5->add_press_call([&](Button *button, Press press, ConfigDraw *config) {

        });

        auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
        if(button2.get()!=nullptr)
            *viwers_container << std::move(button) << std::move(button2) << std::move(button4) << std::move(button5);
        else
            *viwers_container << std::move(button) << std::move(button4) << std::move(button5);

        viwers_container->set_color(SK_ColorTRANSPARENT);

        return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Container> Application::main_page()
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
					draw->stack_page->stack(option_overlay());
				}
			} });

        auto minimage_container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
        *minimage_container << std::move(lminipage);
        return minimage_container;
    }
