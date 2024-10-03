#define _USE_MATH_DEFINES
#include <cmath>
#include <nlohmann/json.hpp>

#include <random>
#include "MessageProcessing.h"
#include "utils/Reader.h"
#include "utils/FileStructures.h"

std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning, curan::ui::IconResources &resources)
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

std::unique_ptr<curan::ui::Overlay> success_overlay(const std::string &success, curan::ui::IconResources &resources)
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
    button_record_pose->add_press_call([processing, &resources](Button *button, Press press, ConfigDraw *config)
                                       {
            if(config!=nullptr && config->stack_page!=nullptr)
                config->stack_page->stack(success_overlay("recorded word point",resources));
            button->set_waiting_color(SK_ColorCYAN);
            processing->should_record_point_from_world = true; });

    auto button_record_calibrate = Button::make("Record Calib Pose", resources);
    button_record_calibrate->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(300, 180));
    button_record_calibrate->add_press_call([&processing, &resources](Button *button, Press press, ConfigDraw *config)
                                            {
            if(config!=nullptr && config->stack_page!=nullptr)
                config->stack_page->stack(success_overlay("recorded calibration point",resources));
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
        solve_registration->add_press_call([processing, &resources](Button *button, Press press, ConfigDraw *config)
                                           {
            Eigen::Matrix<double,3,Eigen::Dynamic> moving_points = processing->landmarks;

            auto points = processing->world_points();
            if (!points){
                if(config!=nullptr && config->stack_page!=nullptr)
                    config->stack_page->stack(warning_overlay("no landmark collected",resources));
                return;
            }

            Eigen::Matrix<double,3,Eigen::Dynamic> fixed_points = *points;

            Eigen::Matrix<double,3,1> centroid_fixed_points = fixed_points.rowwise().mean();
            Eigen::Matrix<double,3,1> centroid_moving_points = moving_points.rowwise().mean();

            Eigen::Matrix<double,3,Eigen::Dynamic> centered_fixed_points = fixed_points;
            Eigen::Matrix<double,3,Eigen::Dynamic> centered_moving_points = moving_points;

            centered_fixed_points.colwise() -= centroid_fixed_points;
            centered_moving_points.colwise() -= centroid_moving_points;

            Eigen::Matrix<double,3,3> H = Eigen::Matrix<double,3,3>::Zero();

            if(centered_fixed_points.cols()!=centered_moving_points.cols()){
                if(config!=nullptr && config->stack_page!=nullptr)
                    config->stack_page->stack(warning_overlay("number of fixed and moving landmarks must be the same",resources));    
                return;
            }
            
            for(size_t i = 0; i < centered_fixed_points.cols(); ++i)
                H+=centered_fixed_points.col(i)*centered_moving_points.col(i).transpose();

            Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd( H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<double,3,3> rotation_fixed_to_moving = svd.matrixV()* svd.matrixU().transpose();
            auto determinant = rotation_fixed_to_moving.determinant();
            if(determinant < 1-1e-7 || determinant> 1+1e-7){
                if(config!=nullptr && config->stack_page!=nullptr)
                    config->stack_page->stack(warning_overlay("estimated rotation matrix has -1 determinant",resources));    
                return;
            }

            Eigen::Matrix<double, 3, 1> translation_fixed_to_moving = centroid_moving_points - rotation_fixed_to_moving * centroid_fixed_points;

            Eigen::Matrix<double, 4, 4> registration_solution = Eigen::Matrix<double, 4, 4>::Identity();
            registration_solution.block<3, 3>(0, 0) = rotation_fixed_to_moving.transpose();
            registration_solution.block<3, 1>(0, 3) = -rotation_fixed_to_moving.transpose() * translation_fixed_to_moving;

            Eigen::Matrix<double, 3, Eigen::Dynamic> aligned_moving_points =  moving_points;

            for(size_t col = 0; col <aligned_moving_points.cols() ; ++col)
                aligned_moving_points.col(col) = (rotation_fixed_to_moving.transpose()*aligned_moving_points.col(col)- rotation_fixed_to_moving.transpose() * translation_fixed_to_moving).eval();
            
            auto error = (aligned_moving_points-fixed_points).array().square().rowwise().sum().sqrt().colwise().sum();

            if(config!=nullptr && config->stack_page!=nullptr)
                config->stack_page->stack(success_overlay("performed registration",resources));    
            std::cout << "registration error: " << error << std::endl; 
            
            processing->registration_solution = std::make_tuple(registration_solution,error(0,0));
            
            });
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

    if (argc > 2)
    {
        std::cout << "you can only pass single json file that optionaly specifies\n"
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
    double calibration_error;
    Eigen::Matrix<double, 4, 4> needle_calibration = Eigen::Matrix<double, 4, 4>::Identity();
    bool specified_landmarks = false;
    Eigen::Matrix<double, 3, Eigen::Dynamic> landmarks;

    if (argc > 1)
    {
        std::ifstream in(argv[1]);
        if (!in.is_open())
        {
            std::cout << "failure to read the specification file";
            return 1;
        }
        nlohmann::json needle_calibration_specification;
        in >> needle_calibration_specification;
        if (needle_calibration_specification.contains("previous_calibration_file_path"))
        {
            previous_calibration = true;
            std::cout << "reading previous calibration...\n";
            std::string path_output_location = needle_calibration_specification["previous_calibration_file_path"];
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
            auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');

            for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
                for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
                    needle_calibration(row, col) = calibration_matrix(row, col);
            std::cout << "using calibration:\n"
                      << needle_calibration << std::endl;
        }
        else
        {
            std::cout << "no previous calibration was specified\n";
        }

        if (needle_calibration_specification.contains("landmarks_to_register_path"))
        {
            specified_landmarks = true;
            std::cout << "reading landmarks to register\n";
            std::string path_output_location = needle_calibration_specification["landmarks_to_register_path"];
            std::ifstream in{path_output_location};
            if (!in.is_open())
            {
                std::cout << "failure to read previous calibration file\n";
                return 1;
            }
            nlohmann::json json_landmarks;
            in >> json_landmarks;
            std::cout << "parsed json.\n";
            std::string homogenenous_transformation = json_landmarks["landmarks_to_register"];
            std::stringstream stream;
            stream << homogenenous_transformation;
            auto temp_landmarks = curan::utilities::convert_matrix(stream, ',');

            std::cout << "with the homogeneous matrix :\n"
                      << landmarks << std::endl;

            if (temp_landmarks.cols() < 3)
            {
                std::cout << "to solve the registration problem you need at least 3 points\n";
                return 1;
            }

            landmarks = Eigen::Matrix<double, 3, Eigen::Dynamic>::Ones(3, temp_landmarks.cols());

            for (Eigen::Index row = 0; row < temp_landmarks.rows(); ++row)
                for (Eigen::Index col = 0; col < temp_landmarks.cols(); ++col)
                    landmarks(row, col) = temp_landmarks(row, col);
            std::cout << "using landmarks to register:\n"
                      << landmarks << std::endl;
        }
        else
        {
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
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " ", "");

    if (processing->size_calibration_points() != 0)
    {
        curan::utilities::NeedleCalibrationData calibration{date,needle_calibration,processing->calibration_error};
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/needle_calibration.json");
        o << calibration;
        std::cout << "calibration data from needle coordinates" << calibration << std::endl;
    }

    auto points = processing->world_points();
    if (points)
    {
        nlohmann::json needle_poses_recorded_from_world_coordinates;
        needle_poses_recorded_from_world_coordinates["timestamp"] = date;
        std::stringstream ss;
        ss << (*points).format(CommaInitFmt);
        needle_poses_recorded_from_world_coordinates["world_points"] = ss.str();
        // write prettified JSON to another file
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/points_in_world_space.json");
        o << needle_poses_recorded_from_world_coordinates;
        std::cout << "needle poses from world coordinates" << needle_poses_recorded_from_world_coordinates << std::endl;
    }

    if (processing->registration_solution)
    {
        auto [registration_sol, error] = *(processing->registration_solution);
        curan::utilities::RegistrationData registration_data{date,registration_sol,error,curan::utilities::Type::LANDMARK};
        std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/registration.json");
        o << registration_data;
        std::cout << "needle poses from world coordinates" << registration_data << std::endl;
    }

    return 0;
}