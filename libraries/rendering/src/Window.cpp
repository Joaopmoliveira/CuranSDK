#include "rendering/Window.h"

namespace curan {
namespace renderable {

Window::Window(Info& info) {
    traits = vsg::WindowTraits::create();
    traits->windowTitle = info.title;
    traits->debugLayer = info.is_debug;
    traits->apiDumpLayer = info.api_dump;

    std::visit(overloaded{
                [this](bool arg) { traits->fullscreen = true; },
                [this](WindowSize size) { traits->width, traits->height; traits->fullscreen = false; }
        }, info.window_size);

    traits->screenNum = info.screen_number;
    traits->display = info.display;

    resourceHints = vsg::ResourceHints::create();

    root = vsg::Group::create();
    auto newnode = create_bottom();

    root->addChild(newnode);

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
        std::printf("Elapsed time in mili: %d\n", (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }
}

Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable) {
    if (!renderable->obj_contained)
        return ref;
    if (!ref.window) {
        std::cout << "failed to get window\n";
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