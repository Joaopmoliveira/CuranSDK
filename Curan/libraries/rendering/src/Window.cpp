#include "rendering/Window.h"
#include "utils/Overloading.h"
#include "utils/Logger.h"
#include "rendering/Floor.h"
#include <filesystem>


namespace curan {
namespace renderable {

Window::Window(Info& info) : number_of_images{5} {
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

    window = vsg::Window::create(traits);
    if (!window)
        throw std::runtime_error("Could not create window");

    viewer = vsg::Viewer::create();
    
    viewer->addWindow(window);

    vsg::ComputeBounds computeBounds;
    root_plus_floor->accept(computeBounds);

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
    
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(.5, .5, .5);
    ambientLight->intensity = 0.01f;
   
    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "directional";
    directionalLight->color.set(.5f, .5f, .5f);
    directionalLight->intensity = 1.0f;
    directionalLight->direction.set(0.0, 0.0, -1.0);
    
    view->addChild(directionalLight);
    view->addChild(ambientLight);
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

    if (!resourceHints)
    {
        resourceHints = vsg::ResourceHints::create();
        resourceHints->numDescriptorSets = 256;
        resourceHints->descriptorPoolSizes.push_back(VkDescriptorPoolSize{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256 });
    }

    viewer->compile(resourceHints);
    if(run_once())
        number_of_images = window->getSwapchain()->getImageViews().size()+1;
}

Window::~Window() {

}

bool Window::run_once() {
    std::lock_guard<std::mutex> g{mut};
    auto iter = deleted_resource_manager.begin();
    while(iter!=deleted_resource_manager.end()){
        --(*iter).first;
        if((*iter).first<1){
            iter = deleted_resource_manager.erase(iter);
        }
        else
            ++iter;
    }
    bool val = viewer->advanceToNextFrame();
    if(!val)
        return val;
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
        std::lock_guard<std::mutex> g{mut};
        auto iter = deleted_resource_manager.begin();
        while(iter!=deleted_resource_manager.end()){
            --(*iter).first;
            if((*iter).first<1){
                iter = deleted_resource_manager.erase(iter);
            }
            else
                ++iter;
        }
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
        auto end = std::chrono::steady_clock::now();
        //std::printf("Elapsed time in mili: %d \n",(int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }
}

bool Window::erase(vsg::ref_ptr<Renderable> renderable_to_delete){
    // first we need to check if the identifier exists, once this is done we eliminate it from the map and from the vector
    // of children of the vector in the root 
    std::lock_guard<std::mutex> g{mut};
    auto search = contained_objects.end();
    auto iter = contained_objects.begin();
    while(iter != contained_objects.end()){
        if((*iter).second == renderable_to_delete){
            search = iter;
            break;
        }
    }

    if (search == contained_objects.end())
        return false;
    //the resources might still be in use in previous frames in flight, therefore we store them 
    //in a list that will keep them alive until we have looped through all the frames that were using them
    //in the past    
    // frame [0] -current (delete the resource from scene graph) frame[1] still using it ... frame[number_of_images] still using it...
    deleted_resource_manager.push_back({number_of_images,search->second});
    //we remove all the frames from both the internal map and the scene graph from vsg to guarantee that they are no
    //longer used for the next frames that come due

    auto to_delete = root->children.end();
    for(auto ite = root->children.begin(); ite < root->children.end(); ++ite){
        // in the scene graph, what actually gets attached is
        // the homogeneous transformation of each renderable, not the object itself, thus we
        // compare each homogeneous transformation with the one stored in the previous map
        if(search->second->transform == *ite){ 
            to_delete = ite;
            break;
        }
    }
    contained_objects.erase(search);
    if(to_delete!=root->children.end())
        root->children.erase(to_delete);
    return true;
}

bool Window::erase(const std::string& identifier){
    // first we need to check if the identifier exists, once this is done we eliminate it from the map and from the vector
    // of children of the vector in the root 
    std::lock_guard<std::mutex> g{mut};
    auto search = contained_objects.find(identifier);
    if (search == contained_objects.end())
        return false;
    //the resources might still be in use in previous frames in flight, therefore we store them 
    //in a list that will keep them alive until we have looped through all the frames that were using them
    //in the past    
    // frame [0] -current (delete the resource from scene graph) frame[1] still using it ... frame[number_of_images] still using it...
    deleted_resource_manager.push_back({number_of_images,search->second});
    //we remove all the frames from both the internal map and the scene graph from vsg to guarantee that they are no
    //longer used for the next frames that come due

    auto to_delete = root->children.end();
    for(auto ite = root->children.begin(); ite < root->children.end(); ++ite){
        // in the scene graph, what actually gets attached is
        // the homogeneous transformation of each renderable, not the object itself, thus we
        // compare each homogeneous transformation with the one stored in the previous map
        if(search->second->transform == *ite){ 
            to_delete = ite;
            break;
        }
    }
    contained_objects.erase(search);
    if(to_delete!=root->children.end())
        root->children.erase(to_delete);
    return true;
}

Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable) {
    std::lock_guard<std::mutex> g{ref.mut};
    if (!renderable->obj_contained)
        return ref;
    if (!ref.window) {
        utilities::cout << "failed to get window\n";
        return ref;
    }
    vsg::observer_ptr<vsg::Viewer> observer_viewer(ref.viewer);
    renderable->partial_async_attachment({ observer_viewer,ref.root });
    auto ok = ref.contained_objects.insert({ renderable->identifier(),renderable }).second;
    if (!ok)
        throw std::runtime_error("inserted object that already exists");
    ref.viewer->addUpdateOperation(renderable);
    return ref;
};
}
}