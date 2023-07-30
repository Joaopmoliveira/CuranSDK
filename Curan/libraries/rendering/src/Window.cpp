#include "rendering/Window.h"
#include "utils/Overloading.h"
#include "utils/Logger.h"
#include "rendering/Floor.h"
#include <filesystem>


namespace curan {
namespace renderable {

Window::Window(Info& info) {
    traits = vsg::WindowTraits::create();
    traits->windowTitle = info.title;
    traits->debugLayer = info.is_debug;
    traits->apiDumpLayer = info.api_dump;

    std::visit(utilities::overloaded{
                [this](bool arg) { traits->fullscreen = true; },
                [this](WindowSize size) { traits->width, traits->height; traits->fullscreen = false; }
        }, info.window_size);

    traits->screenNum = info.screen_number;
    traits->display = info.display;
    traits->samples = VK_SAMPLE_COUNT_8_BIT;
    traits->vulkanVersion = VK_API_VERSION_1_2;

    resourceHints = vsg::ResourceHints::create();

    root = vsg::Group::create();
    root_plus_floor = vsg::Group::create();
    
    auto newnode = create_wired_floor();
    root_plus_floor->addChild(newnode);
    root_plus_floor->addChild(root);

    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0, 1.0, 1.0);
    ambientLight->intensity = 0.01f;
    root->addChild(ambientLight);

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "directional";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = 0.4f;
    directionalLight->direction.set(0.0, 0.0, -1.0);
    root->addChild(directionalLight);

    window = vsg::Window::create(traits);
    if (!window)
        throw std::runtime_error("Could not create windows");

    viewer = vsg::Viewer::create();
    viewer->addWindow(window);

    vsg::ComputeBounds computeBounds;
    root->accept(computeBounds);

    vsg::dvec3 centre = vsg::dvec3(0.0,0.0,0.0);
    double radius = 10;
    double nearFarRatio = 0.001;

    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(3.5, -3.5, 3.5), centre, vsg::dvec3(.0, .0, 1.0));
    perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 4.5);

    viewportState = vsg::ViewportState::create(window->extent2D());
    camera = vsg::Camera::create(perspective, lookAt, viewportState);


    // The commandGraph will contain a 2 stage renderGraph 1) 3D scene 2) ImGui (by default also includes clear depth buffers)
    commandGraph = vsg::CommandGraph::create(window);
    auto renderGraph = vsg::RenderGraph::create(window);
    commandGraph->addChild(renderGraph);

    // create the normal 3D view of the scene
    auto view = vsg::View::create(camera);
    view->addChild(vsg::createHeadlight());
    view->addChild(root_plus_floor);

    renderGraph->addChild(view);

    if(info.imgui_interface){
        ImGui::CreateContext();
        auto renderImGui = vsgImGui::RenderImGui::create(window, *info.imgui_interface);
        renderGraph->addChild(renderImGui);
        viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());
    }

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});


    //commandGraph = vsg::createCommandGraphForView(window, camera, root_plus_floor);
    //viewer->assignRecordAndSubmitTaskAndPresentation({ commandGraph });


    if (!resourceHints)
    {
        resourceHints = vsg::ResourceHints::create();
        resourceHints->numDescriptorSets = 256;
        resourceHints->descriptorPoolSizes.push_back(VkDescriptorPoolSize{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256 });
    }

    viewer->compile(resourceHints);
}

Window::~Window() {

}

bool Window::run_once() {
    bool val = viewer->advanceToNextFrame();
    viewer->handleEvents();
    viewer->update();
    viewer->recordAndSubmit();
    viewer->present();
    return val;
}

void Window::run() {
    // rendering main loop
    while (viewer->advanceToNextFrame()) {
        auto start = std::chrono::steady_clock::now();
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
        auto end = std::chrono::steady_clock::now();
        //std::printf("Elapsed time in mili: %d \n",(int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }
}

Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable) {
    if (!renderable->obj_contained)
        return ref;
    if (!ref.window) {
        utilities::cout << "failed to get window\n";
        return ref;
    }
    vsg::observer_ptr<vsg::Viewer> observer_viewer(ref.viewer);
    renderable->partial_async_attachment({ observer_viewer,ref.root });
    auto ok = ref.contained_objects.insert({ renderable->identifier,renderable }).second;
    if (!ok)
        throw std::runtime_error("inserted object that already exists");
    ref.viewer->addUpdateOperation(renderable);
    return ref;
};
}
}