#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include "rendering/Sphere.h"
#include "rendering/Volume.h"
#include "robotutils/RobotModel.h"

#include <iostream>
#include "utils/TheadPool.h"
#include "utils/Reader.h"
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
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
#include "communication/ProtoIGTL.h"

#include "utils/FileStructures.h"

class RobotState
{
    std::atomic<bool> commit_senpuko = false;

public:
    curan::renderable::Window &window_pointer;

    vsg::ref_ptr<curan::renderable::Renderable> robot;
    vsg::ref_ptr<curan::renderable::Renderable> needle_tip;
    vsg::dmat4 needle_calibration;

    RobotState(curan::renderable::Window &wind) : window_pointer{wind}
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
};

bool process_joint_message(RobotState &state, const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    if (er)
        return true;
    for (size_t joint_index = 0; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
        state.robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, message->angles[joint_index]);

    static curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);
    
    static curan::robotic::State internal_state;
    internal_state.sampleTime = sample_time.count();
    double time = 0;
    
    internal_state.q = message->angles;
    
    robot_model.update(internal_state);
    auto robot_to_world = robot_model.homogenenous_transformation();

    vsg::dmat4 homogeneous_transformation;
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            homogeneous_transformation(col, row) = robot_to_world(row,col);

    auto product = homogeneous_transformation*state.needle_calibration;

    if (state.needle_tip.get() != nullptr)
        state.needle_tip->cast<curan::renderable::Sphere>()->update_transform(vsg::translate(product(3,0),product(3,1),product(3,2)));
    return false;
};

int communication(RobotState &state, asio::io_context &context)
{
    asio::ip::tcp::resolver fri_resolver(context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(context, fri_resolver.resolve("172.31.1.148", std::to_string(50010)));
    fri_client->connect([&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message){
    try{ process_joint_message(state, protocol_defined_val, er, message); }
    catch (...){ std::cout << "Exception was thrown" << std::endl; }
    });
    context.run();
    std::cout << "stopped connecting to client\n";
    return 0;
}

void append_needle_tip_with_calibration(RobotState &state)
{
    curan::utilities::NeedleCalibrationData needle_calibration_data{CURAN_COPIED_RESOURCE_PATH"/needle_calibration.json"};

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.01f, 0.0, 0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0, 0.01f, 0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0, 0.0, 0.01f);
    infosphere.stateInfo.blending = true;
    state.needle_tip = curan::renderable::Sphere::make(infosphere);

    for (Eigen::Index row = 0; row < needle_calibration_data.needle_calibration().rows(); ++row)
        for (Eigen::Index col = 0; col < needle_calibration_data.needle_calibration().cols(); ++col)
            state.needle_calibration(col, row) = needle_calibration_data.needle_calibration()(row, col);

    state.needle_tip->update_transform(state.needle_calibration);
    state.window_pointer << state.needle_tip;
}

template <typename itkImage>
void updateBaseTexture3D(vsg::floatArray3D &image, typename itkImage::Pointer out)
{
    typename itk::ImageRegionIteratorWithIndex<itkImage> outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
        image.set(outputIt.GetIndex()[0], outputIt.GetIndex()[1], outputIt.GetIndex()[2], outputIt.Get());
    image.dirty();
}

Eigen::Matrix<double, 4, 4> append_ct_registered_volume_to_scene(RobotState &state, const std::string &path_to_moving_image)
{
    curan::utilities::RegistrationData registration_data{CURAN_COPIED_RESOURCE_PATH"/registration_specification.json"};

    auto fixedImageReader = itk::ImageFileReader<itk::Image<double, 3>>::New();
    fixedImageReader->SetFileName(path_to_moving_image);

    // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
    rescale->SetInput(fixedImageReader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(1.0);

    rescale->Update();

    itk::Image<double, 3>::Pointer output = rescale->GetOutput();
    Eigen::Matrix<double,4,4> original_image_location = Eigen::Matrix<double,4,4>::Identity();
    original_image_location(0,3) = output->GetOrigin()[0]*1e-3;
    original_image_location(1,3) = output->GetOrigin()[1]*1e-3;
    original_image_location(2,3) = output->GetOrigin()[2]*1e-3;

    auto direction = output->GetDirection();
    for(size_t r = 0; r < 3; ++r)
        for(size_t c = 0; c < 3; ++c)
            original_image_location(r,c) = direction(r,c);

    Eigen::Matrix<double,4,4> transformed = registration_data.moving_to_fixed_transform()*original_image_location;

    vsg::dmat4 registration_matrix;

    for (Eigen::Index row = 0; row < 4; ++row)
        for (Eigen::Index col = 0; col < 4; ++col)
            registration_matrix(col, row) = transformed(row, col);

    auto region = output->GetLargestPossibleRegion();
    auto size_itk = region.GetSize();
    auto spacing = output->GetSpacing();

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size_itk.GetSize()[0];
    volumeinfo.height = size_itk.GetSize()[1];
    volumeinfo.depth = size_itk.GetSize()[2];
    volumeinfo.spacing_x = spacing[0];
    volumeinfo.spacing_y = spacing[1];
    volumeinfo.spacing_z = spacing[2];
    auto volume = curan::renderable::Volume::make(volumeinfo);
    state.window_pointer << volume;

    volume->cast<curan::renderable::Volume>()->update_volume([=](vsg::floatArray3D &image){ updateBaseTexture3D<itk::Image<double, 3>>(image, output); });
    volume->cast<curan::renderable::Volume>()->update_transform(registration_matrix);
    return registration_data.moving_to_fixed_transform();
}

void append_desired_trajectory_data(RobotState &state)
{
    curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json"};

    Eigen::Matrix<double, 4, 1> vectorized_eigen_entry = Eigen::Matrix<double, 4, 1>::Ones();
    Eigen::Matrix<double, 4, 1> desired_target_point = Eigen::Matrix<double, 4, 1>::Ones();

    vectorized_eigen_entry.block<3,1>(0,0) = trajectory_data.entry()*1e-3;
    desired_target_point.block<3,1>(0,0) = trajectory_data.target()*1e-3;;

    auto registration_matrix = append_ct_registered_volume_to_scene(state, trajectory_data.path_to_original_image());

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.01f, 0.0, 0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0, 0.01f, 0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0, 0.0, 0.01f);
    infosphere.stateInfo.blending = true;

    { // we define a point for the entry of the brain
        infosphere.geomInfo.color = vsg::vec4(0.0, 1.0, 0.0, 1.0);
        auto entry_point = curan::renderable::Sphere::make(infosphere);
        auto entry_point_in_world_coordiantes = registration_matrix*vectorized_eigen_entry;
        entry_point->cast<curan::renderable::Sphere>()->update_transform(vsg::translate(entry_point_in_world_coordiantes[0],entry_point_in_world_coordiantes[1],entry_point_in_world_coordiantes[2]));
        state.window_pointer << entry_point;
    }

    { // we define a point for the target of the brain
        infosphere.geomInfo.color = vsg::vec4(0.0, 0.0, 1.0, 1.0);
        auto target = curan::renderable::Sphere::make(infosphere);
        auto target_point_in_world_coordiantes = registration_matrix*desired_target_point;
        target->cast<curan::renderable::Sphere>()->update_transform(vsg::translate(target_point_in_world_coordiantes[0],target_point_in_world_coordiantes[1],target_point_in_world_coordiantes[2]));
        state.window_pointer << target;
    }
}

int main(int argc, char **argv)
{
    auto pool = curan::utilities::ThreadPool::create(3);
    asio::io_context context;

    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "Intra Operative Nagivation";
    curan::renderable::Window::WindowSize size{2000, 1800};
    info.window_size = size;
    curan::renderable::Window window{info};

    RobotState robot_state{window};

    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    create_info.number_of_links = 8;
    robot_state.robot = curan::renderable::SequencialLinks::make(create_info);
    window << robot_state.robot;

    append_needle_tip_with_calibration(robot_state);
    append_desired_trajectory_data(robot_state);

    pool->submit(curan::utilities::Job{"communication with robot", [&](){ communication(robot_state, context); }});

    window.run();
    context.stop();
    return 0;
}
