#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/MiniPage.h"
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

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#include <Eigen/Dense>

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

enum PanelType{
    ORIGINAL_VOLUME = 0,
    RESAMPLED_VOLUME,
    TRAJECTORY_ORIENTED_VOLUME,
    NUMBER_OF_VOLUMES  
};

struct DataSpecificApplication
{
	bool is_acpc_being_defined = false;
    bool is_trajectory_being_visualized = false;

    std::array<curan::ui::VolumetricMask,PanelType::NUMBER_OF_VOLUMES> map;
    PanelType current_volume = PanelType::ORIGINAL_VOLUME;

	curan::ui::IconResources &resources;

	Panels current_panel_arragement = Panels::ONE_PANEL;
	
    std::optional<Eigen::Matrix<double,3,1>> ac_point;
	std::optional<Eigen::Matrix<double,3,1>> pc_point;
	std::optional<Eigen::Matrix<double,3,1>> midline;

	curan::ui::MiniPage *minipage = nullptr;

	DataSpecificApplication(ImageType::Pointer volume, curan::ui::IconResources &in_resources) : resources{in_resources}, map{{{volume},{volume},{volume}}}
	{

	}

	std::unique_ptr<curan::ui::Overlay> create_overlay_with_warning(const std::string& warning){
		using namespace curan::ui;
		auto warn = Button::make(" ","warning.png", resources);
		warn->set_click_color(SK_AlphaTRANSPARENT).set_hover_color(SK_AlphaTRANSPARENT).set_waiting_color(SK_AlphaTRANSPARENT).set_size(SkRect::MakeWH(400, 200));

		auto button = Button::make(warning, resources);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 50));

		auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*viwers_container << std::move(warn) << std::move(button);
		viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0,.8,1.0});

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
                                    ImageType::PointType ac_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,ac_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lac_point = Eigen::Matrix<double,3,1>::Zero();
                                    lac_point(0,0) = ac_point_in_world_coordinates[0];
                                    lac_point(1,0) = ac_point_in_world_coordinates[1];
                                    lac_point(2,0) = ac_point_in_world_coordinates[2];
                                    ac_point = lac_point;
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
			button_cp->add_textdefined_callback([&](MutatingTextPanel *button, const std::string &str, ConfigDraw *config){
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
                                    ImageType::PointType pc_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,pc_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lpc_point = Eigen::Matrix<double,3,1>::Zero();
                                    lpc_point(0,0) = pc_point_in_world_coordinates[0];
                                    lpc_point(1,0) = pc_point_in_world_coordinates[1];
                                    lpc_point(2,0) = pc_point_in_world_coordinates[2];
                                    pc_point = lpc_point;
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
			button_cp->add_textdefined_callback([&](MutatingTextPanel *button, const std::string &str, ConfigDraw *config){
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
                                    ImageType::PointType midpoint_point_in_world_coordinates;
                                    map[current_volume].get_volume()->TransformIndexToPhysicalPoint(local_index,midpoint_point_in_world_coordinates);
                                    Eigen::Matrix<double,3,1> lmid_point = Eigen::Matrix<double,3,1>::Zero();
                                    lmid_point(0,0) = midpoint_point_in_world_coordinates[0];
                                    lmid_point(1,0) = midpoint_point_in_world_coordinates[1];
                                    lmid_point(2,0) = midpoint_point_in_world_coordinates[2];
                                    midline = lmid_point;
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
                [this](Button *button, Press press, ConfigDraw *config){
                    if(!midline || !ac_point || !pc_point){
					    if(config->stack_page!=nullptr){
						    config->stack_page->stack(create_overlay_with_warning("you must specify all points to resample the volume"));
				        }
                    }
                }
            );
			*text_container << std::move(button_ac) << std::move(button_cp) << std::move(button_midpoint) << std::move(perform_resampling);
			auto total_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
			*total_container << std::move(text_container) << std::move(viwers_container);
			total_container->set_divisions({0.0, 0.1, 1.0});
			minipage->construct(std::move(total_container), SK_ColorBLACK);
		}
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
		button3->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
		button3->add_press_call([&](Button *button, Press press, ConfigDraw *config) {

		});

		auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
		*viwers_container << std::move(button) << std::move(button2) << std::move(button3);
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

		auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/mri_brain");
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