#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "utils/Flag.h"
#include "utils/TheadPool.h"
#include "rendering/PhaseWiredBox.h"
#include <asio.hpp>
#include <cassert>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>
#include "rendering/Box.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char* argv[]){
    //initualize the thread pool;
    auto projeto = curan::utilities::ThreadPool::create(4);

    std::string serial_connection_name = std::string(CURAN_SERIAL_PORT);
    if(serial_connection_name.size()==0){
        if(argc<2){
            std::cout << "To use this service please provide the port number of the serial connection \neg. windows will be COM ports, linux will use /dev/ttyS* or /dev/ttyUSB*, etc\n" ;
            return 1;
        }
        serial_connection_name = std::string(argv[1]);
    }
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open(serial_connection_name);

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

    curan::renderable::Box::Info infobox;
    infobox.builder = vsg::Builder::create();
    infobox.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    infobox.geomInfo.dx = vsg::vec3(.1f,0.0,0.0);
    infobox.geomInfo.dy = vsg::vec3(0.0,.1f,0.0);
    infobox.geomInfo.dz = vsg::vec3(0.0,0.0,.1f);
    infobox.geomInfo.position = vsg::vec3(1.0,1.0,0.05f);
    auto box = curan::renderable::Box::make(infobox);
    window << box;

    curan::utilities::Job append_box;
	append_box.description = "function that adds the wired box on screen";
	append_box.function_to_execute = [&box,&serial]() {
        char to_send = 10;
        size_t nread = 0;
        for (;;) {
            asio::write(serial,asio::buffer(&to_send,1));
            asio::streambuf input_buffer;
            nread = asio::read_until(
                serial, input_buffer, 'e'
            );
            std::istream is(&input_buffer);
            std::string line;
            std::getline(is, line);
            if(line.size()>0){
                line.pop_back();
            }
            if(line.size()>0){
                int result{};
                auto [ptr, ec] = std::from_chars(line.data(), line.data() + line.size(), result);
                if (ec == std::errc()){
                    double distance = result;
                    if(distance<40){
                        auto mat = vsg::scale(1.0,1.0,(distance/10.0)+1.0);
                        box->update_transform(mat);
                    }else {
                        auto mat = vsg::scale(1.0,1.0,(40/10.0)+1.0);
                        box->update_transform(mat);
                    }
                    std::cout << distance << '\n';
                }
            }
        }
	};
    projeto->submit(append_box);

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
            q_current[i] = std::sin(5*time)*1.57;
		    iiwa->q[i] = q_current[i];
            robotRenderableCasted->set(i,q_current[i]);
	    }
        static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->qDot[i] = (iiwa->q[i] - q_old[i]) / sampletime;
	    }   

        robot->getMassMatrix(iiwa->M,iiwa->q);
	    iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,NUMBER_OF_JOINTS);              // 3x1 position of flange (body = 7), expressed in base coordinates

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
        time += sampletime;
    }
    serial.close();
    return 0;
}