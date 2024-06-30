#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"

#include "robotutils/RobotModel.h"

constexpr size_t n_joints = 7;

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
    infosphere.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.02f,0.0,0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0,0.02f,0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0,0.0,0.02f);
    infosphere.stateInfo.blending = true;
    auto sphere = curan::renderable::Sphere::make(infosphere);
    auto mat = vsg::translate(0.0,0.0,0.0);
    sphere->update_transform(mat);
    window << sphere;

    curan::robotic::RobotModel<7> robot_model{"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
    auto robotRenderableCasted = robotRenderable->cast<curan::renderable::SequencialLinks>();

    double q_current [n_joints] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double sampletime = 0.001;
    double time = 0.0;

    curan::robotic::State state;

    while(window.run_once()) {
	    for (int i = 0; i < n_joints; i++) {
            state.q[i] = std::sin(10*time)*1.57;
            robotRenderableCasted->set(i,q_current[i]);
	    }
        mat = vsg::translate(robot_model.translation()(0,0),robot_model.translation()(1,0),robot_model.translation()(2,0));
        sphere->update_transform(mat);
        time += sampletime;
    }

    return 0;
}