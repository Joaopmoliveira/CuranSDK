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

        switch (current_volume){
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