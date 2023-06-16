#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "link_demo.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"

int render(std::shared_ptr<SharedRobotState> state)
{
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
    window << robotRenderable;

    curan::renderable::DynamicTexture::Info infotexture;
    infotexture.height = 100;
    infotexture.width = 100;
    infotexture.geomInfo.dx = vsg::vec3(0.2f,0.0f,0.0f);
    infotexture.geomInfo.dy = vsg::vec3(0.0f,.2f,0.0f);
    infotexture.geomInfo.dz = vsg::vec3(0.0f,0.0f,0.0f);
    infotexture.geomInfo.position = vsg::vec3(0.0f,0.1f,0.0f);
    infotexture.builder = vsg::Builder::create();
    auto dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
    dynamic_texture->update_transform(vsg::rotate<double>(vsg::radians(90.0),1.0,0.0,0.0)*vsg::translate<double>(0.0,0.126,0.0));
    robotRenderable->append(dynamic_texture);

    float value = 1.0;
    auto updateBaseTexture = [value](vsg::vec4Array2D& image)
    {
        using value_type = typename vsg::vec4Array2D::value_type;
        for (size_t r = 0; r < image.height(); ++r)
        {
            float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
            value_type* ptr = &image.at(0, r);
            for (size_t c = 0; c < image.width(); ++c)
            {
                float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                vsg::vec2 delta((r_ratio - 0.5f), (c_ratio - 0.5f));

                float angle = atan2(delta.x, delta.y);

                float distance_from_center = vsg::length(delta);

                float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0f * value) + 1.0f) * 0.5f;

                ptr->r = intensity;
                ptr->g = intensity;
                ptr->b = intensity;
                ptr->a = 1.0f;

                ++ptr;
            }
        }
    };
    dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);

    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here
	
	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	// Attach tool
	double toolMass = 0.0;                                                                     // No tool for now
	RigidBodyDynamics::Math::Vector3d toolCOM = RigidBodyDynamics::Math::Vector3d::Zero(3, 1);
	RigidBodyDynamics::Math::Matrix3d toolInertia = RigidBodyDynamics::Math::Matrix3d::Zero(3, 3);
	auto myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);

	robot->attachToolToRobotModel(myTool.get());


    RigidBodyDynamics::Math::VectorNd measured_torque = RigidBodyDynamics::Math::VectorNd::Zero(7,1);
    Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
    Vector3d p_0_cur = Vector3d(0, 0, 0.045);
    RigidBodyDynamics::Math::MatrixNd Jacobian = RigidBodyDynamics::Math::MatrixNd::Zero(6, NUMBER_OF_JOINTS);

    auto robotRenderableCasted = robotRenderable->cast<curan::renderable::SequencialLinks>();

    while(window.run_once() && !state->should_kill_myself()) {
        auto current_reading = state->read();
        auto q_current = current_reading.getMeasuredJointPosition();
        auto tau_current = current_reading.getExternalTorque();

	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->q[i] = q_current[i];
            robotRenderableCasted->set(i,q_current[i]);
		    measured_torque[i] = tau_current[i];
	    }
        static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->qDot[i] = (q_current[i] - q_old[i]) / current_reading.getSampleTime();
	    }   

        robot->getMassMatrix(iiwa->M,iiwa->q);
	    iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,7);              // 3x1 position of flange (body = 7), expressed in base coordinates

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
    }

    return 0;
}
