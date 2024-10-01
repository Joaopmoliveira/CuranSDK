

int main(int argc, char **argv)
{
    asio::io_context context;
    ApplicationState* app_pointer = nullptr;

    curan::renderable::ImGUIInterface::Info info_gui{[pointer_to_address = &app_pointer](vsg::CommandBuffer &cb){interface(cb,pointer_to_address);}};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "Volume Reconstructor";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{2000, 1800};
    info.window_size = size;
    curan::renderable::Window window{info};

    ApplicationState application_state{window};
    app_pointer = &application_state;
    
    nlohmann::json calibration_data;
    std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json");

    if(!in.is_open()){
        std::cout << "failure to open configuration file\n";
        return 1;
    }

    in >> calibration_data;
    std::string timestamp = calibration_data["timestamp"];
    std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
    double error = calibration_data["optimization_error"];
    std::printf("Using calibration with average error of : %f\n on the date ", error);
    std::cout << timestamp << std::endl;
    std::stringstream matrix_strm;
    matrix_strm << homogenenous_transformation;
    auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');
    std::cout << "with the homogeneous matrix :\n"
              << calibration_matrix << std::endl;
    for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
        for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
            application_state.robot_state.calibration_matrix(col, row) = calibration_matrix(row, col);
    
   
    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    application_state.robot_state.robot = curan::renderable::SequencialLinks::make(create_info);
    window << application_state.robot_state.robot;

    application_state.pool->submit(curan::utilities::Job{"communication with robot",[&](){communication(application_state.robot_state,context);}});
    application_state.pool->submit(curan::utilities::Job{"reconstruct volume",[&](){
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(8);
        while(!context.stopped()){
            if(application_state.robot_state.inject_frame() && application_state.robot_state.integrated_volume.get()!=nullptr)
                application_state.robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
        }
    }});

    window.run();
    context.stop();
    return 0;
}
