#include "rendering/Robot.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = true;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/kukamedglb/arm.json";
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks<7>::make(robot_path);
        robotRenderable->update_transform(vsg::MatrixTransform::create(vsg::translate(vsg::dvec3(0.0, 0.0, 0.43))));
        window << robotRenderable;

        std::atomic<bool> continue_running = true;
        auto lambda_called = [&continue_running,robotRenderable](){
            auto robot = robotRenderable.cast<curan::renderable::SequencialLinks<7>>();
            std::array<double,7> robot_config = {0.0};
            std::array<double,7> step = {0.01,0.01,0.0,0.00,0.00,0.00,0.00};
            static_assert(step.size()==robot_config.size());
            while(continue_running.load()){
                size_t index = 0;
                for(auto& angle : robot_config){
                    angle += step[index];
                    ++index;
                }
                robot->set<0>(robot_config[0]);
                robot->set<1>(robot_config[1]);
                robot->set<2>(robot_config[2]);
                robot->set<3>(robot_config[3]);
                robot->set<4>(robot_config[4]);
                robot->set<5>(robot_config[5]);
                robot->set<6>(robot_config[6]);
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        };
        //std::thread local_robot_mover{lambda_called};

        window.run();
        continue_running.store(false);
        //local_robot_mover.join();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map)
                    std::cout << "Object contained: " << p.first << '\n';
            });

    } catch (const vsg::Exception &ve) {
        for (int i = 0; i < argc; ++i)
            std::cerr << argv[i] << " ";
        std::cerr << "\n[Exception] - " << ve.message
                  << " result = " << ve.result << std::endl;
        return 1;
    }
    return 0;
}
