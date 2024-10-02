#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>
#include "utils/TheadPool.h"
#include "utils/Reader.h"
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Box.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "imgui_stdlib.h"
#include <map>
#include <string>


class RobotState{
    std::atomic<bool> commit_senpuko = false;
    std::atomic<bool> f_record_frame = false;
    std::atomic<bool> f_regenerate_integrated_reconstructor_frame = false;
    std::atomic<bool> f_inject_frame = false;

public:
    vsg::ref_ptr<curan::renderable::Renderable> robot;
    curan::renderable::Window &window_pointer;
    curan::image::BoundingBox4Reconstruction box_class;
    vsg::ref_ptr<curan::renderable::Renderable> rendered_box;
    curan::image::IntegratedReconstructor::Info integrated_volume_create_info;
    vsg::ref_ptr<curan::renderable::Renderable> integrated_volume;
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> dynamic_texture;
    vsg::dmat4 calibration_matrix;
    

    RobotState(curan::renderable::Window &wind) : window_pointer{wind},integrated_volume_create_info{{{0.1,0.1,0.1}}, {{0,0,0}}, {{10,10,10}}, {{{1,0,0},{0,1,0},{0,0,1}}}}
    {
    }

    RobotState(const RobotState &) = delete;
    RobotState(RobotState &&) = delete;
    RobotState &operator=(const RobotState &) = delete;
    RobotState &operator=(RobotState &&) = delete;

    operator bool()
    {
        return commit_senpuko.load();
    }

    void operator()(bool value)
    {
        commit_senpuko.store(value);
    }

    enum RecordStatus
    {
        START_RECORDING_FOR_BOX_UPDATE,
        NOT_RECORDING
    };

    void record_frames(RecordStatus status)
    {
        //std::cout << "flag \"record_frames\" updated!!!";
        f_record_frame.store(status == START_RECORDING_FOR_BOX_UPDATE);
    }

    bool record_frames()
    {
        return f_record_frame.load();
    }

    enum GenerateStatus
    {
        GENERATE_VOLUME,
        ALREADY_GENERATED
    };

    void generate_volume(GenerateStatus status)
    {
        //std::cout << "flag \"generate_volume\" updated!!!";
        f_regenerate_integrated_reconstructor_frame.store(status == GENERATE_VOLUME);
    }

    bool generate_volume()
    {
        return f_regenerate_integrated_reconstructor_frame.load();
    }

    enum InjectVolumeStatus
    {
        INJECT_FRAME,
        FREEZE_VOLUME
    };

    void inject_frame(InjectVolumeStatus status)
    {
        //std::cout << "flag \"inject_frame\" updated!!!";
        f_inject_frame.store(status == INJECT_FRAME);
    }

    bool inject_frame()
    {
        return f_inject_frame.load();
    }
};

int communication(RobotState &state, asio::io_context &context)
{
    asio::ip::tcp::resolver resolver(context);
    auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(context,resolver.resolve("172.31.1.148", std::to_string(50000)));

    auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
    {
        try
        {
            if (process_message(state, protocol_defined_val, er, val) || state)
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    client->connect(lam);
    std::cout << "connecting to client\n";
    
    asio::ip::tcp::resolver fri_resolver(context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(context,fri_resolver.resolve("172.31.1.148", std::to_string(50010)));

    auto lam_fri = [&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
    {
        try
        {
            if (process_joint_message(state, protocol_defined_val, er, message))
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    fri_client->connect(lam_fri);
    
    context.run();
    std::cout << "stopped connecting to client\n";
    return 0;
}

int main(int argc, char **argv)
{
    asio::io_context context;

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
    
    nlohmann::json needle_calibration_data;
    std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json");

    if(!in.is_open()){
        std::cout << "failure to open configuration file\n";
        return 1;
    }

    in >> needle_calibration_data;
    std::string timestamp = needle_calibration_data["timestamp"];
    std::string homogenenous_transformation = needle_calibration_data["homogeneous_transformation"];
    double error = needle_calibration_data["optimization_error"];
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
    std::cout << "closed file handle";
    return 0;
}
