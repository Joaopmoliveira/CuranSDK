#define _USE_MATH_DEFINES
#include <cmath>
#include <nlohmann/json.hpp>

#include <random>
#include "MessageProcessing.h"

/*
This executable requires nothing

It produces

*/

curan::ui::Page create_main_page(ConfigurationData &data, std::shared_ptr<ProcessingMessage> &processing, curan::ui::IconResources &resources)
{
    using namespace curan::ui;

    auto start_connection_callback = [&data, processing](Button *button, Press press, ConfigDraw *config)
    {
        if (!processing->connection_status.value())
        {
            curan::utilities::Job val{"connection thread", [processing](){ processing->communicate(); }};
            data.shared_pool->submit(val);
        }
        else
        {
            processing->attempt_stop();
            processing->connection_status.set(false);
        }
    };

	processing = std::make_shared<ProcessingMessage>(data);
	processing->port = data.port;

    auto start_connection = Button::make("Connect", resources);
    start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorRED).set_size(SkRect::MakeWH(300, 180));
    start_connection->add_press_call(start_connection_callback);


    auto button_record_pose = Button::make("Record Pose", resources);
    button_record_pose->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_record_pose->add_press_call([processing](Button *button, Press press, ConfigDraw *config)
        {
            button->set_waiting_color(SK_ColorCYAN);
            processing->should_record_point_from_world = true;
        }
    );

    auto button_record_calibrate = Button::make("Record Calib Pose", resources);
    button_record_calibrate->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_record_calibrate->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config) {
        button->set_waiting_color(SK_ColorCYAN);
        processing->should_record_point_for_calibration = true;
    }); 

    auto button_trigger_calibration = Button::make("Calibrate", resources);
    button_trigger_calibration->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_trigger_calibration->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config) {
        if(processing->calibrate_needle())
            button->set_waiting_color(SK_ColorGREEN);
        else
            button->set_waiting_color(SK_ColorRED);
    });

    processing->connection_button = start_connection.get();
    processing->record_calib_button = button_record_calibrate.get();
    processing->record_world_button = button_record_pose.get();

    auto image = resources.get_icon("hr_repeating.png");
	std::unique_ptr<curan::ui::Container> buttoncontainer;
	if (image)
		buttoncontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL, *image);
	else
		buttoncontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL);
    *buttoncontainer << std::move(start_connection) << std::move(button_record_pose) << std::move(button_record_calibrate) << std::move(button_trigger_calibration);
    return Page{std::move(buttoncontainer), SK_ColorBLACK};
}

int main(int argc, char *argv[])
{
    using namespace curan::ui;

	curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};

    ConfigurationData data;
    data.shared_pool = curan::utilities::ThreadPool::create(4);
    std::cout << "the received port is: " << data.port << "\n";
    std::unique_ptr<Context> context = std::make_unique<Context>();
    DisplayParams param{std::move(context)};
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));


    std::shared_ptr<ProcessingMessage> processing;
    auto page = create_main_page(data, processing, resources);

    page.update_page(viewer.get());

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
    processing->attempt_stop();
    std::cout << "trying to stop communication"<< std::endl;

    std::printf("\nRememeber that you always need to\nperform the temporal calibration before attempting the\nspacial calibration! Produced JSON file:\n");

    auto return_current_time_and_date = []()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        return ss.str();
    };

    {
        nlohmann::json calibration_data;
        calibration_data["timestamp"] = return_current_time_and_date();
        std::stringstream ss;
        ss << processing->needle_calibration;
        calibration_data["needle_homogeneous_transformation"] = ss.str();
        calibration_data["optimization_error"] = processing->calibration_error;
        // write prettified JSON to another file
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/needle_calibration.json");
        o << calibration_data;
        std::cout<< "calibration data from needle coordinates" << calibration_data << std::endl;
    }

    {
        nlohmann::json needle_poses_recorded_from_world_coordinates;
        needle_poses_recorded_from_world_coordinates["timestamp"] = return_current_time_and_date();
        std::stringstream ss;
        ss << processing->world_points();
        needle_poses_recorded_from_world_coordinates["world_points"] = ss.str();
        // write prettified JSON to another file
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/points_in_world_space.json");
        o << needle_poses_recorded_from_world_coordinates;
        std::cout << "needle poses from world coordinates" << needle_poses_recorded_from_world_coordinates << std::endl;
    }
    return 0;
}