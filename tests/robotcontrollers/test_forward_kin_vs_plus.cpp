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
#include "imgui_stdlib.h"
#include <map>
#include <string>
#include "utils/FileStructures.h"
#include "robotutils/RobotModel.h"

/*
This executable requires:

1. Temporal calibration has been performed on the robot
which translates into a file called temporal_calibration.json
with the date at which the calibration was performed

2. Spatial calibration has been performed on the robot
which translates into a file called spatial_calibration.json
with the date at which the calibration was performed

And it outputs

1. 

*/

std::mutex mut;
Eigen::Matrix<double,4,4> local_transform_from_plus = Eigen::Matrix<double,4,4>::Identity();
Eigen::Matrix<double,4,4> local_transform_from_kinematics = Eigen::Matrix<double,4,4>::Identity();

bool process_image_message(igtl::MessageBase::Pointer val)
{
    igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
    message_body->Copy(val);
    int c = message_body->Unpack(1);
    if (!(c & igtl::MessageHeader::UNPACK_BODY))
        return false; // failed to unpack message, therefore returning without doing anything

    igtl::Matrix4x4 image_transform;
    message_body->GetMatrix(image_transform);
    Eigen::Matrix<double,4,4> local_transform;
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            local_transform(row, col) = image_transform[row][col];

    local_transform(0, 3) *= 1e-3;
    local_transform(1, 3) *= 1e-3;
    local_transform(2, 3) *= 1e-3;

    std::lock_guard<std::mutex> g{mut};
    local_transform_from_plus = local_transform;
    //std::cout << "transform from plus:" << homogeneous_transformation << std::endl;
    return true;
}

std::map<std::string, std::function<bool(igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
    {"IMAGE", process_image_message}};

bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
    assert(val.IsNotNull());
    if (er)
        return true;
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(val);
    return false;
}



bool process_joint_message(const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    if (er)
        return true;
    static curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);
    static curan::robotic::State state;
    state.sampleTime = 0.001;
    state.q = message->angles;
    robot_model.update(state);
    std::lock_guard<std::mutex> g{mut};
    local_transform_from_kinematics = robot_model.homogenenous_transformation();
    return false;   
}

int communication(asio::io_context &context)
{
    asio::ip::tcp::resolver resolver(context);
    auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(context,resolver.resolve("localhost", std::to_string(18944)));

    auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
    {
        try
        {
            if (process_message(protocol_defined_val, er, val))
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
            if (process_joint_message(protocol_defined_val, er, message))
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
    auto pool = curan::utilities::ThreadPool::create(2);
    pool->submit(curan::utilities::Job{"communication with robot",[&](){communication(context);}});
    auto guard = asio::make_work_guard(context);
    while(!context.stopped()){
        
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
        std::lock_guard<std::mutex> g{mut};
        auto val = local_transform_from_kinematics-local_transform_from_plus;
        std::cout << (local_transform_from_kinematics.block<3,3>(0,0).transpose()*val.block<3,1>(0,3)).transpose() << std::endl;
    }
    context.stop();
    return 0;
}
