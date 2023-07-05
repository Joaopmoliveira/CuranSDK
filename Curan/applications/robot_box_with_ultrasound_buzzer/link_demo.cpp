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

int render(std::shared_ptr<SharedRobotState> state)
{
    //initualize the thread pool;
	curan::utilities::initialize_thread_pool(4);

    std::string serial_connection_name = std::string(CURAN_SERIAL_PORT);
    if(serial_connection_name.size()==0){
        throw std::runtime_error("the size cannot be zero");
    }
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open(serial_connection_name);

    curan::utilities::initialize_thread_pool(4);
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

    std::array<float,3> temp_pos = {0.0,0.0,0.0};
    std::atomic<std::array<float,3>> current_position;
    current_position.store(temp_pos);
    std::atomic<double> current_distance = 0.0;
    curan::utilities::Job  update_box_size;
	update_box_size.description = "function that adds the wired box on screen";
	update_box_size.function_to_execute = [&box,&serial,&current_distance]() {
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
                    double distance = (result<40.0) ? result : 40.0;
                    auto mat = vsg::scale(1.0,1.0,(distance/10.0)+1.0);
                    box->update_transform(mat);
                    current_distance.store(distance);
                }
            }
        }
        current_distance = 40.0;
	};
    curan::utilities::pool->submit(update_box_size);
    
    curan::utilities::Job append_box;
	append_box.description = "function that adds the wired box on screen";
	append_box.function_to_execute = [&window,&current_position,&current_distance]() {
        auto box = curan::renderable::PhaseWiredBox::make();
        window << box;
        auto casted_box = box->cast<curan::renderable::PhaseWiredBox>();
        std::array<float,3> temp_pos = current_position.load();
        std::cout << "fixing first vertex\n";
        bool already_recording = false;
        std::chrono::steady_clock::time_point start;
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            vsg::vec3 origin(temp_pos[0],temp_pos[1],temp_pos[2]);
            casted_box->update_frame(origin);
            //std::cout << current_distance.load() << std::endl;
            if(current_distance.load()<7.0){
                if(!already_recording){
                    already_recording = true;
                    start = std::chrono::steady_clock::now();
                    continue;
                }
                std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count()>std::chrono::milliseconds(1000).count())
                    break;
            } else {
            already_recording = false;
            }
        }
        already_recording = false;
        vsg::vec3 origin_fixed = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("first vertex - x : %f y : %f z : %f\n",origin_fixed[0],origin_fixed[1],origin_fixed[2]);
        std::cout << "fixing second vertex\n";
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            vsg::vec3 origin(temp_pos[0],temp_pos[1],temp_pos[2]);
            casted_box->update_frame(origin_fixed,origin);
            if(current_distance.load()<7.0){
                if(!already_recording){
                    already_recording = true;
                    start = std::chrono::steady_clock::now();
                    continue;
                }
                std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count()>std::chrono::milliseconds(1000).count())
                    break;
            } else {
            already_recording = false;
            }
        }
        already_recording = false;
        auto xdir = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("second vertex - x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        std::cout << "fixing third vertex\n";
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            vsg::vec3 origin(temp_pos[0],temp_pos[1],temp_pos[2]);
            casted_box->update_frame(origin_fixed,xdir,origin);
            if(current_distance.load()<7.0){
                if(!already_recording){
                    already_recording = true;
                    start = std::chrono::steady_clock::now();
                    continue;
                }
                std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count()>std::chrono::milliseconds(1000).count())
                    break;
            } else {
            already_recording = false;
            }
        }
        already_recording = false;
        auto ydir = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("third vertex - x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        std::cout << "fixing fourth vertex\n";
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            vsg::vec3 origin(temp_pos[0],temp_pos[1],temp_pos[2]);
            casted_box->update_frame(origin_fixed,xdir,ydir,origin);
            if(current_distance.load()<7.0){
                if(!already_recording){
                    already_recording = true;
                    start = std::chrono::steady_clock::now();
                    continue;
                }
                std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count()>std::chrono::milliseconds(1000).count())
                    break;
            } else {
            already_recording = false;
            }
        }
        already_recording = false;
        auto zdir = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("fourth vertex - x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        casted_box->print(origin_fixed,xdir,ydir,zdir);
	};
    curan::utilities::pool->submit(append_box);

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

        temp_pos[0] = (float)p_0_cur(0,0);
        temp_pos[1] = (float)p_0_cur(1,0);
        temp_pos[2] = (float)p_0_cur(2,0);
        current_position.store(temp_pos);

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
    }
    serial.close();
    curan::utilities::terminate_thread_pool();
    return 0;
}
