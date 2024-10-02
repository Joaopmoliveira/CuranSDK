#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include "rendering/Sphere.h"
#include "rendering/Volume.h"

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
#include "communication/ProtoIGTL.h"

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

bool process_tracking_message(RobotState &state, igtl::MessageBase::Pointer val)
{
    igtl::TrackingDataMessage::Pointer trackingData;
    trackingData = igtl::TrackingDataMessage::New();
    trackingData->Copy(val);
    int c = trackingData->Unpack(1);
    if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
        int nElements = trackingData->GetNumberOfTrackingDataElements();
        if (nElements != 1)
            throw std::runtime_error("there are not enough tracking elements in the incoming message");
        igtl::TrackingDataElement::Pointer trackingElement;
        trackingData->GetTrackingDataElement(0, trackingElement);

        igtl::Matrix4x4 image_transform;
        trackingElement->GetMatrix(image_transform);

        vsg::dmat4 homogeneous_transformation;
        for (size_t row = 0; row < 4; ++row)
            for (size_t col = 0; col < 4; ++col)
                homogeneous_transformation(col, row) = image_transform[row][col];

        homogeneous_transformation(3, 0) *= 1e-3;
        homogeneous_transformation(3, 1) *= 1e-3;
        homogeneous_transformation(3, 2) *= 1e-3;

        auto product = homogeneous_transformation*state.needle_calibration;

        if (state.needle_tip.get() != nullptr)
            state.needle_tip->cast<curan::renderable::Sphere>()->update_transform(vsg::translate(product(3,0),product(3,1),product(3,2)));

        return true;
    }
    return false;
}

std::map<std::string, std::function<bool(RobotState &state, igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
    {"TDATA", process_tracking_message}};

bool process_message(RobotState &state, size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
    assert(val.IsNotNull());
    if (er)
        return true;
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(state, val);
    return false;
}

bool process_joint_message(RobotState &state, const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    if (er)
        return true;
    for (size_t joint_index = 0; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
    {
        state.robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, message->angles[joint_index]);
    }
    return false;
};

int communication(RobotState &state, asio::io_context &context)
{
    asio::ip::tcp::resolver resolver(context);
    auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(context, resolver.resolve("localhost", std::to_string(50000)));

    auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
    {
        try
        {
            process_message(state, protocol_defined_val, er, val);
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    client->connect(lam);
    std::cout << "connecting to client\n";

    igtl::StartTrackingDataMessage::Pointer startTrackingMsg = igtl::StartTrackingDataMessage::New();
	startTrackingMsg->SetDeviceName("ROBOT");
	startTrackingMsg->SetResolution(40);
	startTrackingMsg->SetCoordinateName("Base");
	startTrackingMsg->Pack();
	auto to_send = curan::utilities::CaptureBuffer::make_shared(startTrackingMsg->GetPackPointer(), startTrackingMsg->GetPackSize(), startTrackingMsg);
	client->write(to_send);

    asio::ip::tcp::resolver fri_resolver(context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(context, fri_resolver.resolve("localhost", std::to_string(50010)));

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

void append_needle_tip_with_calibration(RobotState &state)
{
    nlohmann::json needle_calibration_data;
    std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/needle_calibration.json");

    if (!in.is_open())
    {
        std::cout << "failure to open needle calibration configuration file\n";
        std::terminate();
    }

    in >> needle_calibration_data;
    std::string timestamp = needle_calibration_data["timestamp"];
    std::string homogenenous_transformation = needle_calibration_data["needle_homogeneous_transformation"];
    double error = needle_calibration_data["optimization_error"];
    std::printf("Using calibration with average error of : %f\n on the date ", error);
    std::cout << timestamp << std::endl;
    std::stringstream matrix_strm;
    matrix_strm << homogenenous_transformation;
    auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');
    std::cout << "with the homogeneous matrix :\n"
              << calibration_matrix << std::endl;

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.01f, 0.0, 0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0, 0.01f, 0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0, 0.0, 0.01f);
    infosphere.stateInfo.blending = true;
    state.needle_tip = curan::renderable::Sphere::make(infosphere);

    for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
        for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
            state.needle_calibration(col, row) = calibration_matrix(row, col);

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
    nlohmann::json registration_data;
    std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/registration.json");

    if (!in.is_open())
    {
        std::cout << "failure to open needle calibration configuration file" << std::endl;
        std::terminate();
    } else {
        std::cout << "file is open" << std::endl;
    }

    in >> registration_data;

    std::string type = registration_data["type"];
    std::cout << "registration type: " << type << std::endl;
    std::string timestamp = registration_data["timestamp"];
    std::cout << "timestamp : " << timestamp << std::endl;
    std::string homogenenous_transformation = registration_data["moving_to_fixed_transform"];
    double error = registration_data["registration_error"];
    std::cout << "using registration estimated using: " << type << std::endl;
    std::printf("Using registration with average error of : %f\n on the date ", error);
    std::cout << timestamp << std::endl;
    std::stringstream matrix_strm;
    matrix_strm << homogenenous_transformation;
    auto registration_mat = curan::utilities::convert_matrix(matrix_strm, ',');
    std::cout << "with the homogeneous matrix :\n"<< registration_mat << std::endl;

    vsg::dmat4 registration_matrix;

    for (Eigen::Index row = 0; row < registration_mat.rows(); ++row)
        for (Eigen::Index col = 0; col < registration_mat.cols(); ++col)
            registration_matrix(col, row) = registration_mat(row, col);

    auto fixedImageReader = itk::ImageFileReader<itk::Image<double, 3>>::New();
    fixedImageReader->SetFileName(path_to_moving_image);

    // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
    rescale->SetInput(fixedImageReader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(1.0);

    rescale->Update();

    itk::Image<double, 3>::Pointer output = rescale->GetOutput();
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
    return registration_mat;
}

void append_desired_trajectory_data(RobotState &state)
{
    nlohmann::json trajectory_data;
    std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json");
    if (!in.is_open())
    {
        std::cout << "failure to find the trajectory specification file";
        std::terminate();
    }
    
    in >> trajectory_data;

    std::cout << "using json with data:\n" << trajectory_data << std::endl;

    std::stringstream ss;
    std::string target = trajectory_data["target"];
    std::cout << "string target:\n" << target << std::endl;
    ss << target;
    auto eigen_target = curan::utilities::convert_matrix(ss,',');
    std::cout << "target:" << eigen_target << std::endl;
    ss = std::stringstream{};
    std::string entry = trajectory_data["entry"];
    std::cout << "string target:\n" << entry << std::endl;
    ss << entry;
    auto eigen_entry = curan::utilities::convert_matrix(ss,',');
    std::cout << "entry:" << eigen_entry << std::endl;
    assert(eigen_target.cols() == 1 && eigen_target.rows() == 3);
    assert(eigen_entry.cols() == 1 && eigen_entry.rows() == 3);
    Eigen::Matrix<double, 4, 1> vectorized_eigen_entry = Eigen::Matrix<double, 4, 1>::Ones();
    Eigen::Matrix<double, 4, 1> desired_target_point = Eigen::Matrix<double, 4, 1>::Ones();
    for (size_t i = 0; i < 3; ++i)
    {
        desired_target_point[i] = eigen_target(i, 0);
        vectorized_eigen_entry[i] = eigen_entry(i, 0);
    }

    std::cout << "getting path....";
    std::string path_to_moving_image = trajectory_data["moving_image_directory"];
    std::cout << path_to_moving_image << std::endl;

    auto registration_matrix = append_ct_registered_volume_to_scene(state, path_to_moving_image);

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.01f, 0.0, 0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0, 0.01f, 0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0, 0.0, 0.01f);
    infosphere.stateInfo.blending = true;

    { // we define a point for the entry of the brain
        auto entry_point = curan::renderable::Sphere::make(infosphere);
        auto entry_point_in_world_coordiantes = registration_matrix*vectorized_eigen_entry;
        entry_point->cast<curan::renderable::Sphere>()->update_transform(vsg::translate(entry_point_in_world_coordiantes[0],entry_point_in_world_coordiantes[1],entry_point_in_world_coordiantes[2]));
        state.window_pointer << entry_point;
    }

    { // we define a point for the target of the brain
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

    std::cout << "appending needle tip\n";
    append_needle_tip_with_calibration(robot_state);
    std::cout << "appending desired trajectory\n";
    append_desired_trajectory_data(robot_state);

    std::cout << "appended everything\n";

    pool->submit(curan::utilities::Job{"communication with robot", [&]()
                                       { communication(robot_state, context); }});

    window.run();
    context.stop();
    return 0;
}
