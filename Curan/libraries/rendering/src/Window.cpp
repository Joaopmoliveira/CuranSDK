#include "rendering/Window.h"
#include "utils/Overloading.h"
#include "utils/Logger.h"
#include <filesystem>

namespace curan {
namespace renderable {

vsg::ref_ptr<vsg::Node> create_bottom()
{
    auto builder = vsg::Builder::create();

    auto scene = vsg::Group::create();
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());

    std::filesystem::path path_to_texture = CURAN_COPIED_RESOURCE_PATH"/base_pattern/CheckerBoardSeemlessPattern.jpg";
    auto textureData = vsg::read_cast<vsg::Data>(path_to_texture.c_str(),options);

    if(textureData) {
        utilities::cout <<  "Failed to load the checkered pattern\n";
    }

    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    stateInfo.two_sided = true;
    stateInfo.image = textureData;
    geomInfo.position.set(0.0, 0.0, 0);
    geomInfo.dx.set(10, 0.0, 0.0);
    geomInfo.dy.set(0.0, 10, 0.0);

    scene->addChild(builder->createQuad(geomInfo, stateInfo));

    return scene;
}

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

    resourceHints = vsg::ResourceHints::create();

    root = vsg::Group::create();
    auto newnode = create_bottom();

    root->addChild(newnode);

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "directional";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = 1.0f;
    directionalLight->direction.set(0.0, 0.0, -1.0);
    root->addChild(directionalLight);

    window = vsg::Window::create(traits);
    if (!window)
        throw std::runtime_error("Could not create windows");

    viewer = vsg::Viewer::create();
    viewer->addWindow(window);

    vsg::ComputeBounds computeBounds;
    root->accept(computeBounds);

    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.001;

    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));
    perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 4.5);

    viewportState = vsg::ViewportState::create(window->extent2D());
    camera = vsg::Camera::create(perspective, lookAt, viewportState);

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));
    commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({ commandGraph });

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
       // utils::cout << "Elapsed time in mili: " << (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() <<"\n";
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