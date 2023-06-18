#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/SequencialLinks.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Sphere.h"

int main(){
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

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    auto sphere = curan::renderable::Sphere::make(infosphere);
    auto mat = vsg::scale(0.01,0.01,0.01);
    sphere->update_transform(mat);
    window << sphere;
    

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

    double q_current [NUMBER_OF_JOINTS] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double sampletime = 0.001;
    double time = 0.0;
    while(window.run_once()) {
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
            q_current[i] = 10*time;
		    iiwa->q[i] = q_current[i];
            robotRenderableCasted->set(i,q_current[i]);
	    }
        static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->qDot[i] = (q_current[i] - q_old[i]) / sampletime;
	    }   

        robot->getMassMatrix(iiwa->M,iiwa->q);
	    iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,7);              // 3x1 position of flange (body = 7), expressed in base coordinates
        
        mat = mat * vsg::translate(p_0_cur(0,0),p_0_cur(1,0),p_0_cur(2,0));
        sphere->update_transform(mat);

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
        time += sampletime;
    }

    return 0;
}