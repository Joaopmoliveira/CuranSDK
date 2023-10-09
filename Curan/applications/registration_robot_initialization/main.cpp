
#include <optional>
#include <nlohmann/json.hpp>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "link_demo.h"

const double pi = std::atan(1) * 4;

void interface(vsg::CommandBuffer &cb, info_solve_registration &registration)
{
    ImGui::Begin("Box Specification Selection");
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    static bool local_record_data = false;
    static bool previous = local_record_data;
    
    if (registration.optimization_running.load())
    {
        ImGui::TextColored(ImVec4{1.0, 0.0, 0.0, 1.0}, "Optimization Currently Running...Please Wait");
        local_record_data = false;
    }
    else{
        ImGui::TextColored(ImVec4{0.0, 1.0, 0.0, 1.0}, "Can initialize solution with the LBR Med");
        ImGui::Checkbox("Start Robot Positioning", &local_record_data);
    }

    if (!local_record_data && local_record_data != previous)
    {
        std::cout << "Initializing the new batch of tasks\n";

        auto homogenenous_transformation = registration.moving_homogenenous.get_matrix();
        itk::Matrix<double, 3, 3> mat_moving;
        mat_moving.SetIdentity();

        for (size_t col = 0; col < 3; ++col)
            for (size_t row = 0; row < 3; ++row)
                mat_moving(row, col) = homogenenous_transformation(row, col);

        registration.moving_image->SetDirection(mat_moving);

        float origin[3];
        origin[0] = homogenenous_transformation(0,3);
        origin[1] = homogenenous_transformation(1,3);
        origin[2] = homogenenous_transformation(2,3);

        registration.moving_image->SetOrigin(origin);
        
        curan::utilities::Job job{};
        job.description = "Execution of registration";
        job.function_to_execute = [&](){
            registration.optimization_running.store(true);
            registration.full_runs.emplace_back(solve_registration(registration));
            registration.optimization_running.store(false);
        };
        registration.thread_pool->submit(job); 
    }

    registration.robot_client_commands_volume_init.store(local_record_data);
    previous = local_record_data;

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}

int main(int argc, char **argv)
{
    auto fixedImageReader = FixedImageReaderType::New();
    auto movingImageReader = MovingImageReaderType::New();

    fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH "/reconstruction_results.mha");
    movingImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha");

    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
    }
    catch (...)
    {
        std::string error_name = "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n" + std::string(CURAN_COPIED_RESOURCE_PATH);
        std::printf(error_name.c_str());
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();
    ImageType::Pointer pointer2movingimage = movingImageReader->GetOutput();

    // now is the trickie part, I think the best strategy is to launch a threadpool and submit the registration algorithm
    auto thread_pool = curan::utilities::ThreadPool::create(2);
    CurrentInitialPose initial_pose;
    std::vector<std::tuple<double, TransformType::Pointer>> full_runs;
    std::atomic<bool> variable = false;
    std::atomic<bool> robot_client_commands_volume_init = false;

    info_solve_registration registration_shared{
        pointer2fixedimage,
        pointer2movingimage,
        nullptr,
        initial_pose,
        variable,
        robot_client_commands_volume_init,
        thread_pool,
        full_runs,
        nullptr,
        nullptr};

    curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer &cb)
                                                     { interface(cb, registration_shared); }};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.imgui_interface = ui_interface;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    registration_shared.robot_render = curan::renderable::SequencialLinks::make(create_info);
    window << registration_shared.robot_render;

    ImageType::RegionType region_fixed = pointer2fixedimage->GetLargestPossibleRegion();
    ImageType::SizeType size_itk_fixed = region_fixed.GetSize();
    ImageType::SpacingType spacing_fixed = pointer2fixedimage->GetSpacing();

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size_itk_fixed.GetSize()[0];
    volumeinfo.height = size_itk_fixed.GetSize()[1];
    volumeinfo.depth = size_itk_fixed.GetSize()[2];
    volumeinfo.spacing_x = spacing_fixed[0];
    volumeinfo.spacing_y = spacing_fixed[1];
    volumeinfo.spacing_z = spacing_fixed[2];

    auto volume_fixed = curan::renderable::Volume::make(volumeinfo);
    window << volume_fixed;

    auto direction = pointer2fixedimage->GetDirection();
    auto origin = pointer2fixedimage->GetOrigin();
    auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            fixed_homogenenous_transformation(col, row) = direction(row, col);

    volume_fixed->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

    auto casted_volume_fixed = volume_fixed->cast<curan::renderable::Volume>();
    auto updater = [pointer2fixedimage](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer2fixedimage); };
    casted_volume_fixed->update_volume(updater);

    ImageType::RegionType region_moving = pointer2movingimage->GetLargestPossibleRegion();
    ImageType::SizeType size_itk_moving = region_moving.GetSize();
    ImageType::SpacingType spacing_moving = pointer2movingimage->GetSpacing();

    volumeinfo.width = size_itk_moving.GetSize()[0];
    volumeinfo.height = size_itk_moving.GetSize()[1];
    volumeinfo.depth = size_itk_moving.GetSize()[2];
    volumeinfo.spacing_x = spacing_moving[0];
    volumeinfo.spacing_y = spacing_moving[1];
    volumeinfo.spacing_z = spacing_moving[2];

    auto volume_moving = curan::renderable::Volume::make(volumeinfo);
    window << volume_moving;

    auto casted_volume_moving = volume_moving->cast<curan::renderable::Volume>();
    auto updater_moving = [pointer2movingimage](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer2movingimage); };

    casted_volume_moving->update_volume(updater_moving);

    auto moving_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    moving_homogenenous_transformation(3, 0) = pointer2movingimage->GetOrigin()[0] / 1000.0;
    moving_homogenenous_transformation(3, 1) = pointer2movingimage->GetOrigin()[1] / 1000.0;
    moving_homogenenous_transformation(3, 2) = pointer2movingimage->GetOrigin()[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            moving_homogenenous_transformation(col, row) = pointer2movingimage->GetDirection()(row, col);

    casted_volume_moving->update_transform(moving_homogenenous_transformation);

    Eigen::Matrix<double, 4, 4> mat_moving_here = Eigen::Matrix<double, 4, 4>::Identity();
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            mat_moving_here(row, col) = pointer2movingimage->GetDirection()(row, col);

    mat_moving_here(0, 3) = pointer2movingimage->GetOrigin()[0];
    mat_moving_here(1, 3) = pointer2movingimage->GetOrigin()[1];
    mat_moving_here(2, 3) = pointer2movingimage->GetOrigin()[2];

    initial_pose.update_matrix(mat_moving_here);
    registration_shared.volume_moving = casted_volume_moving;
    curan::utilities::Job job{};
    job.description = "Execution of registration";
    job.function_to_execute = [&]()
    {
        variable.store(true);
        full_runs.emplace_back(solve_registration(registration_shared));
        variable.store(false);
    };
    thread_pool->submit(job);
    auto communication_callable = [&]()
    {
        communication(registration_shared);
    };
    std::thread communication_thread(communication_callable);
    window.run();
    communication_thread.join();

    size_t minimum_index = 0;
    size_t current_index = 0;
    double minimum_val = 1e20;
    for (const auto &possible : full_runs)
    {
        if (minimum_val > std::get<0>(possible))
        {
            minimum_index = current_index;
            minimum_val = std::get<0>(possible);
        }
        ++current_index;
    }

    auto finalTransform = std::get<1>(full_runs[minimum_index]);

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            moving_homogenenous_transformation(col, row) = finalTransform->GetMatrix()(row, col);
    moving_homogenenous_transformation(3, 0) = finalTransform->GetOffset()[0];
    moving_homogenenous_transformation(3, 1) = finalTransform->GetOffset()[1];
    moving_homogenenous_transformation(3, 2) = finalTransform->GetOffset()[2];
    casted_volume_moving->update_transform(moving_homogenenous_transformation);

    auto origin_fixed_mine = pointer2fixedimage->GetOrigin();

    TransformType::MatrixType matrix = finalTransform->GetMatrix();
    TransformType::OffsetType offset = finalTransform->GetOffset();

    std::stringstream matrix_value;
    for (size_t y = 0; y < 3; ++y)
    {
        for (size_t x = 0; x < 3; ++x)
        {
            float matrix_entry = matrix[x][y];
            matrix_value << matrix_entry << " ";
        }
        matrix_value << "\n ";
    }

    nlohmann::json registration_transformation;
    registration_transformation["Matrix"] = matrix_value.str();
    registration_transformation["Offset"] = offset;

    std::ofstream output_file{CURAN_COPIED_RESOURCE_PATH "/registration_results.json"};
    output_file << registration_transformation;

    return 0;
}