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
    NUMBER_OF_VOLUMES
};

struct Application
{
    bool are_points_being_defined = false;
    bool is_roi_being_specified = false;
    std::string path;
    std::vector<std::tuple<ImageType::Pointer,std::string>> loaded;
    std::array<curan::ui::VolumetricMask, PanelType::NUMBER_OF_VOLUMES> map;

    PanelType current_volume = PanelType::ORIGINAL_VOLUME;
    Panels current_panel_arragement = Panels::ONE_PANEL;
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(4);

    curan::ui::IconResources &resources;

    bool is_first_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> first_point;
    std::optional<Eigen::Matrix<double, 3, 2>> first_path;
    bool is_second_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> second_point;
    std::optional<Eigen::Matrix<double, 3, 2>> second_path;
    bool is_third_point_being_defined = false;
    std::optional<Eigen::Matrix<double, 3, 1>> third_point;

    curan::ui::MiniPage *minipage = nullptr;

    curan::ui::Button* ptr_button_ac_point = nullptr;
    curan::ui::Button* ptr_button_pc_point = nullptr;
    curan::ui::Button* ptr_button_midpoint = nullptr;

    std::optional<Eigen::Matrix<double, 3, 1>> final_first_point;
    std::optional<Eigen::Matrix<double, 3, 1>> final_second_point;
    std::optional<Eigen::Matrix<double, 3, 1>> final_third_point;

    std::mutex& mut;

    curan::ui::ConfigDraw* ptr_config = nullptr;

    inline void clear_all_paths_and_points(){
        first_point = std::nullopt;
        first_path = std::nullopt;
        second_point = std::nullopt;
        second_path  = std::nullopt;
        third_point = std::nullopt;
    }

    void compute_point(curan::ui::VolumetricMask* vol_mas,const curan::ui::directed_stroke& dir_stroke, curan::ui::ConfigDraw* config);

    Application(curan::ui::IconResources &in_resources,std::string_view path_to_load,std::mutex& mut);

    std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning);

    std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success);

    void view_image_simple();

    void view_roi_selection();

    void point_selection();

    std::unique_ptr<curan::ui::Overlay> layout_overlay();

    std::unique_ptr<curan::ui::Container> main_page();
};

#endif