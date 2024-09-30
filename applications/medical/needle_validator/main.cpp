#define _USE_MATH_DEFINES
#include <cmath>
#include <nlohmann/json.hpp>

#include <random>
#include "MessageProcessing.h"
#include "utils/Reader.h"

curan::ui::Page create_main_page(ConfigurationData &data, std::shared_ptr<ProcessingMessage> &processing, curan::ui::IconResources &resources, bool registration_possible_to_solve = false)
{
    using namespace curan::ui;
    processing = std::make_shared<ProcessingMessage>(data);
    processing->port = data.port;

    auto start_connection_callback = [&data, processing](Button *button, Press press, ConfigDraw *config)
    {
        if (!processing->connection_status.value())
        {
            std::cout << "requesting connection" << std::endl;
            curan::utilities::Job val{"connection thread", [processing]()
                                      { processing->communicate(); }};
            data.shared_pool->submit(val);
        }
        else
        {
            std::cout << "attempting to stop communication" << std::endl;
            processing->attempt_stop();
            processing->connection_status.set(false);
        }
    };

    auto start_connection = Button::make("Connect", resources);
    start_connection->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorRED).set_size(SkRect::MakeWH(300, 180));
    start_connection->add_press_call(start_connection_callback);

    auto button_record_pose = Button::make("Record Pose", resources);
    button_record_pose->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_record_pose->add_press_call([processing](Button *button, Press press, ConfigDraw *config)
                                       {
            button->set_waiting_color(SK_ColorCYAN);
            processing->should_record_point_from_world = true; });

    auto button_record_calibrate = Button::make("Record Calib Pose", resources);
    button_record_calibrate->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_record_calibrate->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config)
                                            {
        button->set_waiting_color(SK_ColorCYAN);
        processing->should_record_point_for_calibration = true; });

    auto button_trigger_calibration = Button::make("Calibrate", resources);
    button_trigger_calibration->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_trigger_calibration->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config)
                                               {
        if(processing->calibrate_needle())
            button->set_waiting_color(SK_ColorGREEN);
        else
            button->set_waiting_color(SK_ColorRED); });

    std::unique_ptr<Button> solve_registration;
    if (registration_possible_to_solve)
    {
        solve_registration = Button::make("Solve Registration", resources);
        solve_registration->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
        solve_registration->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config)
                                           {
            if(processing->calibrate_needle())
                button->set_waiting_color(SK_ColorGREEN);
            else
                button->set_waiting_color(SK_ColorRED); });
    }

    processing->connection_button = start_connection.get();
    processing->record_calib_button = button_record_calibrate.get();
    processing->record_world_button = button_record_pose.get();

    auto image = resources.get_icon("hr_repeating.png");
    std::unique_ptr<curan::ui::Container> buttoncontainer;
    if (image)
        buttoncontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL, *image);
    else
        buttoncontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL);
    if (solve_registration)
        *buttoncontainer << std::move(start_connection) << std::move(button_record_pose) << std::move(button_record_calibrate) << std::move(button_trigger_calibration) << std::move(solve_registration);
    else
        *buttoncontainer << std::move(start_connection) << std::move(button_record_pose) << std::move(button_record_calibrate) << std::move(button_trigger_calibration);
    return Page{std::move(buttoncontainer), SK_ColorBLACK};
}

int main(int argc, char *argv[])
{
    using namespace curan::ui;

    if (argc < 2)
    {
        std::cout << "please pass the name of the calibration file that will be outputed\n";
        return 1;
    }

    if (argc > 4)
    {
        std::cout << "you can only pass the name of the calibration file and single json file that optionaly specifies\n"
                  << "(1) the name of the current calibration you wish to assume\n"
                  << "(2) the name of the file with landmarks\n";
        return 1;
    }

    curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};

    ConfigurationData data;
    data.shared_pool = curan::utilities::ThreadPool::create(4);
    std::cout << "the received port is: " << data.port << "\n";
    std::unique_ptr<Context> context = std::make_unique<Context>();
    DisplayParams param{std::move(context)};
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));


    bool previous_calibration = false;
    double calibration_error ;
    Eigen::Matrix<double,4,4> needle_calibration = Eigen::Matrix<double,4,4>::Identity();
    bool specified_landmarks = false;
    Eigen::Matrix<double, 4, Eigen::Dynamic> landmarks;

    if (argc >= 3)
    {
        nlohmann::json needle_calibration_specification;
        if (needle_calibration_specification.contains("previous_calibration_file"))
        {
            previous_calibration = true;
            std::cout << "reading previous calibration...\n";
            std::string path_output_location = needle_calibration_specification["previous_calibration_file"];
            std::ifstream in{path_output_location};
            if (!in.is_open())
            {
                std::cout << "failure to read previous calibration file\n";
                return 1;
            }
            nlohmann::json calibration_data;
            in >> calibration_data;
            std::cout << "parsed json.\n";
            calibration_error = calibration_data["optimization_error"];

            std::cout << "using calibration with error: " << calibration_error << std::endl;

            std::string homogenenous_transformation = calibration_data["needle_homogeneous_transformation"];

            std::stringstream matrix_strm;
            matrix_strm << homogenenous_transformation;
            std::cout << "string stream: " << matrix_strm.str();
            auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');

            std::cout << "with the homogeneous matrix :\n" << calibration_matrix << std::endl;
            for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
                for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
                    needle_calibration(row, col) = calibration_matrix(row, col);
            std::cout << "using calibration:\n" << needle_calibration << std::endl;
        } else{
            std::cout << "no previous calibration was specified\n";
        }

        if (needle_calibration_specification.contains("landmarks_to_register"))
        {
            specified_landmarks = true;
            std::cout << "reading landmarks to register\n";
            std::string path_output_location = needle_calibration_specification["landmarks_to_register"];
            std::ifstream in{path_output_location};
            if (!in.is_open())
            {
                std::cout << "failure to read previous calibration file\n";
                return 1;
            }
            nlohmann::json json_landmarks;
            in >> json_landmarks;
            std::cout << "parsed json.\n";
            std::string homogenenous_transformation = json_landmarks["landmarks"];
            std::stringstream stream;
            stream << homogenenous_transformation;
            std::cout << "string stream: " << stream.str();
            auto temp_landmarks = curan::utilities::convert_matrix(stream, ',');

            std::cout << "with the homogeneous matrix :\n"
                      << landmarks << std::endl;

            landmarks = Eigen::Matrix<double, 4, Eigen::Dynamic>::Ones(4, temp_landmarks.cols());

            for (Eigen::Index row = 0; row < temp_landmarks.rows(); ++row)
                for (Eigen::Index col = 0; col < temp_landmarks.cols(); ++col)
                    landmarks(row, col) = temp_landmarks(row, col);
            std::cout << "using calibration:\n" << landmarks << std::endl;
        } else{
            std::cout << "no landmark registration was requested\n";
        }
    }

    std::shared_ptr<ProcessingMessage> processing;
    auto page = create_main_page(data, processing, resources, specified_landmarks);
    processing->calibration_error = calibration_error;
    processing->needle_calibration = needle_calibration;
    processing->landmarks = landmarks;

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
    std::cout << "trying to stop communication" << std::endl;

    std::printf("\nRememeber that you always need to\nperform the temporal calibration before attempting the\nspacial calibration! Produced JSON file:\n");

    auto return_current_time_and_date = []()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        return ss.str();
    };

    auto date = return_current_time_and_date();

    if (processing->size_calibration_points() != 0)
    {
        nlohmann::json calibration_data;
        calibration_data["timestamp"] = date;
        std::stringstream ss;
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " ", "");
        ss << processing->needle_calibration;
        calibration_data["needle_homogeneous_transformation"] = ss.str();
        calibration_data["optimization_error"] = processing->calibration_error;
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/needle_calibration.json");
        o << calibration_data;
        std::cout << "calibration data from needle coordinates" << calibration_data << std::endl;
    }

    auto points = processing->world_points();
    if (points)
    {
        nlohmann::json needle_poses_recorded_from_world_coordinates;
        needle_poses_recorded_from_world_coordinates["timestamp"] = date;
        std::stringstream ss;
        ss << *points;
        needle_poses_recorded_from_world_coordinates["world_points"] = ss.str();
        // write prettified JSON to another file
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/points_in_world_space.json");
        o << needle_poses_recorded_from_world_coordinates;
        std::cout << "needle poses from world coordinates" << needle_poses_recorded_from_world_coordinates << std::endl;
    }

    if(processing->registration_solution){
        nlohmann::json registration_data;
	    registration_data["timestamp"] = date;
        std::stringstream ss;
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " ", "");
        auto [registration_sol,error] = *(processing->registration_solution);
        ss << registration_sol;
	    registration_data["moving_to_fixed_transform"] = ss.str();
	    registration_data["registration_error"] = error;
        registration_data["type"] = "landmark";
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/points_in_world_space.json");
        o << registration_data;
        std::cout << "needle poses from world coordinates" << registration_data << std::endl;
    }

    return 0;
}