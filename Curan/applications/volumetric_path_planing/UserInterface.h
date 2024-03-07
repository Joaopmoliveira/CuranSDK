#ifndef USER_INTERFACE_HEADER
#define USER_INTERFACE_HEADER

#include "common_includes.h"


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

struct Application
{
    bool is_acpc_being_defined = false;

    std::array<curan::ui::VolumetricMask, PanelType::NUMBER_OF_VOLUMES> map;

    PanelType current_volume = PanelType::ORIGINAL_VOLUME;
    Panels current_panel_arragement = Panels::ONE_PANEL;
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(4);

    curan::ui::IconResources &resources;

    bool is_first_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> first_point;
    bool is_second_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> second_point;
    bool is_third_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> third_point;

    curan::ui::MiniPage *minipage = nullptr;

    curan::ui::Button* ptr_button_ac_point = nullptr;
    curan::ui::Button* ptr_button_pc_point = nullptr;
    curan::ui::Button* ptr_button_midpoint = nullptr;

    std::optional<Eigen::Matrix<double, 3, 1>> final_first_point;
    std::optional<Eigen::Matrix<double, 3, 1>> final_second_point;
    std::optional<Eigen::Matrix<double, 3, 1>> final_third_point;

    void compute_point(const curan::ui::directed_stroke& dir_stroke, curan::ui::ConfigDraw* config);

    Application(ImageType::Pointer volume, curan::ui::IconResources &in_resources);

    std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning);

    std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success);

    void view_image_simple();

    void view_image_with_point_selection();

    void point_selection();

    std::unique_ptr<curan::ui::Overlay> create_volume_explorer_page();

    std::unique_ptr<curan::ui::Overlay> layout_overlay();

    std::unique_ptr<curan::ui::Overlay> option_overlay();

    std::unique_ptr<curan::ui::Container> main_page();
};

#endif