#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif

#include <type_traits>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <variant>
#include <string>
#include <optional>
#include <ostream>



struct Renderable : public vsg::Inherit<vsg::Operation, Renderable>{

    struct Updatable {
        vsg::observer_ptr<vsg::Viewer> viewer;
        vsg::ref_ptr<vsg::Group> attachmentPoint;

        Updatable(vsg::observer_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Group> attachmentPoint) : viewer{ viewer }, attachmentPoint{ attachmentPoint } 
        {}

        Updatable() 
        {}
    };

    std::string identifier;
    vsg::ref_ptr<vsg::Group> obj_contained;
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    vsg::CompileResult result = vsg::CompileResult{};
    static size_t number;
    Updatable updateinfo;

    Renderable() : identifier{"object" + std::to_string(number)} {
        ++number;
    }

    inline void set_identifier(const std::string& ident) {
        identifier = ident;
    }

    inline void update_transform(const vsg::ref_ptr<vsg::MatrixTransform>& new_transform) {
        transform = new_transform;
    }

    inline void partial_async_attachment(const Updatable& update) {
        updateinfo = update;
        vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;


        if (!updateinfo.attachmentPoint)
            return;

        vsg::ComputeBounds computeBounds;
        obj_contained->accept(computeBounds);

        vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
        double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.5;
        auto scale = vsg::MatrixTransform::create(vsg::scale(1.0 / radius, 1.0 / radius, 1.0 / radius) * vsg::translate(-centre));

        scale->addChild(obj_contained);
        transform->addChild(scale);

        result = ref_viewer->compileManager->compile(obj_contained);
    }

    void run() override{


        vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;
        if (ref_viewer)
            updateViewer(*ref_viewer, result);
        updateinfo.attachmentPoint->addChild(transform);
    }
};

size_t Renderable::number = 0;

struct Sphere : public vsg::Inherit<Renderable, Sphere> {
    struct Info {
        vsg::ref_ptr<vsg::Builder> builder;
        std::optional<std::string> identifier;
    };

    Sphere(Info& info) {
        vsg::dvec3 position(0.0, 0.0, 0.0);
        transform = vsg::MatrixTransform::create(vsg::translate(position));

        obj_contained = vsg::Group::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        auto node = info.builder->createSphere(geomInfo, stateInfo);
        obj_contained->addChild(node);

        if(info.identifier)
            set_identifier(*info.identifier);
    }

    static vsg::ref_ptr<Renderable> make(Info& info) {
        vsg::ref_ptr<Sphere> sphere_to_add = Sphere::create(info);
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }
};

struct Box : public vsg::Inherit<Renderable, Box> {
    struct Info {
        vsg::ref_ptr<vsg::Builder> builder;
        std::optional<std::string> identifier;
    };

    Box(Info& info) {
        vsg::dvec3 position(0.0, 0.0, 0.0);
        transform = vsg::MatrixTransform::create(vsg::translate(position));

        obj_contained = vsg::Group::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        auto node = info.builder->createBox(geomInfo, stateInfo);
        obj_contained->addChild(node);

        if (info.identifier)
            set_identifier(*info.identifier);
    }

    static vsg::ref_ptr<Renderable> make(Info& info) {
        vsg::ref_ptr<Box> sphere_to_add = Box::create(info);
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }
};

struct Cylinder : public vsg::Inherit<Renderable, Cylinder> {
    struct Info {
        vsg::ref_ptr<vsg::Builder> builder;
        std::optional<std::string> identifier;
    };

    Cylinder(Info& info) {
        vsg::dvec3 position(0.0, 0.0, 0.0);
        transform = vsg::MatrixTransform::create(vsg::translate(position));

        obj_contained = vsg::Group::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        auto node = info.builder->createCylinder(geomInfo, stateInfo);
        obj_contained->addChild(node);

        if (info.identifier)
            set_identifier(*info.identifier);
    }

    static vsg::ref_ptr<Renderable> make(Info& info) {
        vsg::ref_ptr<Cylinder> sphere_to_add = Cylinder::create(info);
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }
};

struct Capsule : public vsg::Inherit<Renderable, Capsule> {
    struct Info {
        vsg::ref_ptr<vsg::Builder> builder;
        std::optional<std::string> identifier;
    };

    Capsule(Info& info) {
        vsg::dvec3 position(0.0, 0.0, 0.0);
        transform = vsg::MatrixTransform::create(vsg::translate(position));

        obj_contained = vsg::Group::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        auto node = info.builder->createCapsule(geomInfo, stateInfo);
        obj_contained->addChild(node);

        if (info.identifier)
            set_identifier(*info.identifier);
    }

    static vsg::ref_ptr<Renderable> make(Info& info) {
        vsg::ref_ptr<Capsule> sphere_to_add = Capsule::create(info);
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }
};

struct DynamicTexture : public vsg::Inherit<Renderable, DynamicTexture> {

    size_t width;
    size_t height;
    vsg::ref_ptr<vsg::Data> textureData;

    struct Info {
        size_t width = 100;
        size_t height = 100;
        vsg::ref_ptr<vsg::Builder> builder;
        std::optional<std::string> identifier;
    };

    DynamicTexture(Info& info) : width{info.width}, height{info.height} {
        vsg::dvec3 position(0.0, 0.0, 0.0);
        transform = vsg::MatrixTransform::create(vsg::translate(position));

        obj_contained = vsg::Group::create();

        textureData = vsg::vec4Array2D::create(width, height);
        textureData->properties.dataVariance = vsg::DYNAMIC_DATA; 
        textureData->properties.format = VK_FORMAT_R32G32B32A32_SFLOAT;

        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        stateInfo.two_sided = true;
        stateInfo.greyscale = true;
        stateInfo.image = textureData;
        stateInfo.lighting = true;
        auto node = info.builder->createQuad(geomInfo, stateInfo);

        obj_contained->addChild(node);

        if (info.identifier)
            set_identifier(*info.identifier);
    }

    static vsg::ref_ptr<Renderable> make(Info& info) {
        vsg::ref_ptr<DynamicTexture> sphere_to_add = DynamicTexture::create(info);
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }

    using updater = std::function<void(vsg::vec4Array2D& image)>;

    void update_texture(updater&& update, vsg::ref_ptr<vsg::Viewer> viewer) {
        struct execute_update : public vsg::Visitor {
            updater update;
            vsg::ref_ptr<vsg::Data> data;

            execute_update() {
            
            }

            void set_execution_data(updater&& in_update, vsg::ref_ptr<vsg::Data> in_data) {
                update = std::move(in_update);
                data = in_data;
            }

            // use the vsg::Visitor to safely cast to types handled by the UpdateImage class
            void apply(vsg::vec4Array2D& image) override {
                update(image); 
                image.dirty();
            }

            void operator()()
            {
                data->accept(*this);
            }

        };

        struct async_execute : public vsg::Inherit<vsg::Operation, async_execute> {
            execute_update update_operation;

            async_execute(){
            
            }

            void run() override {
                update_operation();   
            }
        };

        vsg::ref_ptr<async_execute> async_updater = async_execute::create();
        async_updater->update_operation.set_execution_data(std::move(update), textureData);
        viewer->addUpdateOperation(async_updater);        
    }

};

vsg::ref_ptr<vsg::Node> create_bottom()
{
    auto builder = vsg::Builder::create();

    auto scene = vsg::Group::create();

    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    stateInfo.two_sided = true;
    geomInfo.position.set(0.0, 0.0, 0);
    geomInfo.dx.set(3, 0.0, 0.0);
    geomInfo.dy.set(0.0, 3, 0.0);

    scene->addChild(builder->createQuad(geomInfo, stateInfo));

    return scene;
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

struct Window {
    vsg::ref_ptr<vsg::Window> window;
    vsg::ref_ptr<vsg::WindowTraits> traits;
    vsg::ref_ptr<vsg::ResourceHints> resourceHints;
    vsg::ref_ptr<vsg::Group> root;
    vsg::ref_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Camera> camera;
    vsg::ref_ptr<vsg::CommandGraph> commandGraph;
    vsg::ref_ptr<vsg::ProjectionMatrix> perspective;
    vsg::ref_ptr<vsg::ViewportState> viewportState;
    
    std::unordered_map<std::string, vsg::ref_ptr<Renderable>> contained_objects;
public:

    struct WindowSize {
        size_t width = 800;
        size_t height = 800;
    };

    struct Info {
        bool is_debug = false;
        bool api_dump = false;
        int screen_number = 0;
        std::string display = "";
        std::string title = "noname";
        bool full_screen;
        std::variant<bool, WindowSize> window_size;

    };

    Window(Info& info) {
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

    ~Window() {
    
    }

    bool run_once() {
        bool val = viewer->advanceToNextFrame();
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
        return val;
    }

    void run() {
        // rendering main loop
        while (viewer->advanceToNextFrame()){
            auto start = std::chrono::steady_clock::now();
            viewer->handleEvents();
            viewer->update();
            viewer->recordAndSubmit();
            viewer->present();
            auto end = std::chrono::steady_clock::now();
            std::printf("Elapsed time in mili: %d\n", (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        }
    }

    using tranverser = std::function<void(const std::unordered_map<std::string, vsg::ref_ptr<Renderable>>&)>;

    void transverse_identifiers(tranverser&& transv) {
        transv(contained_objects);
    }

    friend Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable);
};

Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable) {
    if (!renderable->obj_contained)
        return ref;
    if (!ref.window) {
        std::cout << "failed to get window\n";
        return ref;
    }
    vsg::observer_ptr<vsg::Viewer> observer_viewer(ref.viewer);
    renderable->partial_async_attachment({ observer_viewer,ref.root});
    auto ok = ref.contained_objects.insert({renderable->identifier,renderable }).second;
    if (!ok)
        throw std::runtime_error("inserted object that already exists");
    ref.viewer->addUpdateOperation(renderable);
    return ref;
};

int main(int argc, char** argv){
    try {
        Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = true;
        info.screen_number = 0;
        info.title = "myviewer";
        Window::WindowSize size{ 1000,800 };
        info.window_size = size;
        Window window{ info };

        vsg::ref_ptr<Renderable> dynamic_texture;

        auto lamb = [&window, &dynamic_texture]() {


            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            auto builder = vsg::Builder::create();
            Sphere::Info infosphere;
            infosphere.builder = builder;
            auto renderable = Sphere::make(infosphere);
            vsg::ComputeBounds bounds;
            window.root->accept(bounds);
            vsg::dvec3 position = { (bounds.bounds.max[0] + bounds.bounds.min[0]) / 2.0, (bounds.bounds.max[1] + bounds.bounds.min[1]) / 2.0 ,(bounds.bounds.max[2] + bounds.bounds.min[2]) / 2.0 + 1.0 };
            auto transform = vsg::MatrixTransform::create(vsg::translate(position));
            renderable->update_transform(transform);
            window << renderable;


            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            DynamicTexture::Info infotexture;
            infotexture.height = 100;
            infotexture.width = 100;
            infotexture.builder = builder;
            dynamic_texture = DynamicTexture::make(infotexture);
            position[0] += 1.0;
            transform = vsg::MatrixTransform::create(vsg::translate(position));
            dynamic_texture->update_transform(transform);

            std::cout << "created a dynamic texture\n";

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

                        float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;

                        ptr->r = intensity;
                        ptr->g = intensity;
                        ptr->b = intensity;
                        ptr->a = 1.0f;

                        ++ptr;
                    }
                }
            };
            dynamic_texture->cast<DynamicTexture>()->update_texture(updateBaseTexture, window.viewer);
            std::cout << "inserting dynamic texture\n";
            window << dynamic_texture;
        };

        std::thread thead(lamb);

        std::atomic<bool> shut_down_updater;
        shut_down_updater.store(false);

        auto updater_of_texture = [&dynamic_texture,&shut_down_updater,&window]() {
            float value = 1.0;
            auto updateBaseTexture = [&value](vsg::vec4Array2D& image)
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

                        float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;

                        ptr->r = intensity;
                        ptr->g = intensity;
                        ptr->b = intensity;
                        ptr->a = 1.0f;

                        ++ptr;
                    }
                }
            };
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            while (!shut_down_updater.load()) {
                dynamic_texture->cast<DynamicTexture>()->update_texture(updateBaseTexture, window.viewer);
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
                value += 0.033;
            }
        };

        std::thread thead2(updater_of_texture);
        
        window.run();

        shut_down_updater.store(true);
        thead.join();
        thead2.join();
        window.transverse_identifiers([](const std::unordered_map<std::string,vsg::ref_ptr<Renderable>>& map) {
            for (auto& p : map)
                std::cout << "Object contained: " << p.first << '\n';
            });

    } catch (const vsg::Exception& ve) {
        for (int i = 0; i < argc; ++i) std::cerr << argv[i] << " ";
        std::cerr << "\n[Exception] - " << ve.message << " result = " << ve.result << std::endl;
        return 1;
    }
    return 0;
}