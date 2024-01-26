#ifndef USER_INTERFACE_HEADER
#define USER_INTERFACE_HEADER

#include "bounding_box.h"

std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(4);

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

    DataSpecificApplication(ImageType::Pointer volume, curan::ui::IconResources &in_resources) : resources{in_resources}, map{{{volume}, nullptr, nullptr}}
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
                    if (config->stack_page != nullptr)
                        config->stack_page->stack(create_overlay_with_success("resampling volume..."));
                    curan::utilities::Job job{"resampling volume", [this, config]()
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
                                                  auto output_bounding_box = bounding_box_original_image.centered_bounding_box<Strategy::CONSERVATIVE, false>(original_eigen_rotation_matrix.transpose() * eigen_rotation_matrix);
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

                                                  try
                                                  {
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
                                                          if (config->stack_page != nullptr)
                                                              config->stack_page->stack(create_overlay_with_warning("failure to process volume"));
                                                          break;
                                                      }
                                                      if (config->stack_page != nullptr)
                                                          config->stack_page->stack(create_overlay_with_success("resampled volume!"));
                                                  }
                                                  catch (...)
                                                  {
                                                      if (config->stack_page != nullptr)
                                                          config->stack_page->stack(create_overlay_with_warning("failed to resample volume to AC-PC"));
                                                  }

                                                  midline = std::nullopt;
                                                  ac_point = std::nullopt;
                                                  pc_point = std::nullopt;
                                              }};
                    pool->submit(job);
                });
            *text_container << std::move(button_ac) << std::move(button_cp) << std::move(button_midpoint) << std::move(perform_resampling);
            auto total_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
            *total_container << std::move(text_container) << std::move(viwers_container);
            total_container->set_divisions({0.0, 0.1, 1.0});
            minipage->construct(std::move(total_container), SK_ColorBLACK);
        }
    }

    std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page()
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
                break;
                default:
                throw std::runtime_error("failed to select the proper index");
                break;
            }
            create_panel_ac_pc_instructions(); });
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
                item_explorer->add(Item{identifier, "volume_" + std::to_string(identifier), buff, extracted_size[0], extracted_size[1]});
            }
            ++identifier;
        }

        item_explorer->set_size(SkRect::MakeWH(800, 400));

        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
        *container << std::move(item_explorer);

        return Overlay::make(std::move(container), SkColorSetARGB(10, 125, 125, 125), true);
    }

    std::unique_ptr<curan::ui::Overlay> create_layout_page()
    {
        using namespace curan::ui;
        if (current_volume == PanelType::TRAJECTORY_ORIENTED_VOLUME)
        {
            auto button = Button::make(" ", "layout1x1.png", resources);
            button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(200, 200));
            button->add_press_call([this](Button *button, Press press, ConfigDraw *config)
                                   { current_panel_arragement = Panels::ONE_PANEL; create_panel_ac_pc_instructions(); });

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

        std::unique_ptr<curan::ui::Button> button2;

        switch (current_volume)
        {
        case PanelType::ORIGINAL_VOLUME:
            button2 = Button::make("Resample AC-PC", resources);
            button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((is_acpc_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
            button2->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                    {
            if(current_volume!=PanelType::ORIGINAL_VOLUME){
                if (config->stack_page != nullptr)
                    config->stack_page->stack(create_overlay_with_warning("cannot resample processed volume"));
                return;
            }
		    is_acpc_being_defined = !is_acpc_being_defined;
		    create_panel_ac_pc_instructions(); });
            case PanelType::RESAMPLED_VOLUME:
            button2 = Button::make("Define Trajectory", resources);
        button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color((is_acpc_being_defined) ? SkColorSetARGB(125, 0x00, 0xFF, 0xFF) : SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
        button2->add_press_call([&](Button *button, Press press, ConfigDraw *config)
                                    {
        if(current_volume!=PanelType::ORIGINAL_VOLUME){
            if (config->stack_page != nullptr)
                config->stack_page->stack(create_overlay_with_warning("cannot resample processed volume"));
            return;
        }
		is_acpc_being_defined = !is_acpc_being_defined;
		create_panel_ac_pc_instructions(); });
        default:

        }

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
        *viwers_container << std::move(button) << std::move(button2) << std::move(button4) << std::move(button5);
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

#endif