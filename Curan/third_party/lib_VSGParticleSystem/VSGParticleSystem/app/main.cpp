#include <iostream>
#include <vsg/all.h>
#include <vsgParticleSystem/all.h>
#include <vsgXchange/all.h>

struct MouseParticles : public vsg::Inherit<vsg::Visitor, MouseParticles> {
    vsgps::Emitter *emitter;
    MouseParticles(vsgps::Emitter &emitter) { this->emitter = &emitter; }

    void apply(vsg::KeyPressEvent &keyPress) override {
        if (keyPress.keyBase == 65451)
            emitter->increaseFactor();
        else if (keyPress.keyBase == 65453)
            emitter->decreaseFactor();
    }
};

int main(int argc, char **argv) {
    std::srand(std::time(nullptr));

    // set up defaults and read command line arguments to override them
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Particle System";

    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    if (arguments.read({"--fullscreen", "--fs"}))
        windowTraits->fullscreen = true;
    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);

    if (arguments.errors())
        return arguments.writeErrorMessages(std::cerr);

    auto basis = vsg::Group::create();
    auto root = vsg::Group::create();
    basis->addChild(root);
    auto builder = vsg::ResolutionBuilder::create();
    builder->options = options;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    size_t numOfObjects = 1000;
    geomInfo.positions = vsg::vec4Array::create(numOfObjects);
    auto colors = vsg::vec4Array::create(numOfObjects);
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);

    vsg::StateInfo stateInfo;
    stateInfo.blending = true;

    vsgps::BuilderProps props{builder, options, &geomInfo, stateInfo, root};
    vsgps::Emitter emitter(props);

    auto dynFile = arguments.value<vsg::Path>(
        "../../data/dynamics/pos-force-demo-gmm8-coupled-3D.json", "--dyn");
    vsgps::Dynamics dynamics = vsgps::Dynamics::fromJson(dynFile);
    std::vector<vsg::dvec3> particlePositions;
    if (!arguments.read({"--ni", "--not-init"})) {
        vsgps::addData(dynFile, props, particlePositions, .1);
    } else {
        vsgps::addData(dynFile, particlePositions, .1);
    }

    double elevation = arguments.value<double>(0, "-z");
    size_t numOfParticles = arguments.value<size_t>(5, "-n");
    vsg::dvec3 centerInit = vsg::dvec3(0, 0, elevation);
    double radiusInit = vsgps::computeRadius(particlePositions, centerInit);

    if (arguments.read({"--random", "-r"})) {
        vsgps::RandomParticleInitializer randomParticleInitializer{
            vsgps::SpatialRange{-4, 4},        vsgps::SpatialRange{-4, 4},
            vsgps::SpatialRange{elevation, 4}, vsgps::SpatialRange{0, 0},
            vsgps::SpatialRange{0, 0},         vsgps::SpatialRange{0, 0}};
        emitter.populate(numOfParticles, randomParticleInitializer);
    } else {
        // vsgps::CircleParticleInitializer
        // circleParticleInitializer{radiusInit,
        //                                                            centerInit};
        // circleParticleInitializer.setVelocityRangeZ({1, 5});
        // circleParticleInitializer.enableVerticalSpace(
        //     arguments.read({"-v", "--vertical"}));
        // emitter.populate(numOfParticles, circleParticleInitializer,
        //                  radiusInit * .075);
        vsgps::ShapeParticleInitializer spi{radiusInit, numOfParticles};
        spi.setElevation(spi.getElevation() *
                         arguments.read({"-v", "--vertical"}));
        emitter.populate(numOfParticles, spi, radiusInit * .05);
    }
    emitter.setStates(dynamics.getVelocityIndexes(), dynamics.getDimension());
    emitter.setNumberOfDiameters(.5);

    vsgps::State eqPoint(dynamics.getDimension());
    eqPoint.setZero();
    double threshold = 1e-5;
    auto emitterCallback = [&](vsgps::State const &xi, vsgps::State &dxi) {
        dxi = (xi - eqPoint).norm() > threshold
                  ? dynamics(xi)
                  : vsgps::State::Zero(dxi.size());
    };
    emitter.setDynamicFunction(emitterCallback);

    vsgps::Axis axis{props};
    axis.fixAxis(!arguments.read({"--free-center", "--fc"}),
                 !arguments.read({"--free-radius", "--fr"}));

    vsgps::Colorbar cb{props};

    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();

    vsg::ref_ptr<vsg::Window> window(vsg::Window::create(windowTraits));
    if (!window) {
        std::cout << "Could not create windows." << std::endl;
        return 1;
    }

    viewer->addWindow(window);

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    basis->accept(computeBounds);
    vsg::dvec3 center =
        (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius =
        vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.0001;

    // set up the camera
    vsg::dvec3 offset{radius * 1.2, radius * 1.2, radius * .5 * 1.2};
    vsg::dvec3 up{0, 0, 1};
    auto lookAt = vsg::LookAt::create(center + offset, center, up);

    vsg::ref_ptr<vsg::Perspective> perspective = vsg::Perspective::create(
        30.0,
        static_cast<double>(window->extent2D().width) /
            static_cast<double>(window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    auto camera = vsg::Camera::create(
        perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));
    viewer->addEventHandler(MouseParticles::create(emitter));

    auto commandGraph = vsg::createCommandGraphForView(window, camera, basis);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));

    // compile all the the Vulkan objects and transfer data required to render
    // the scene
    viewer->compile();

    // rendering main loop
    emitter.tick();
    while (viewer->advanceToNextFrame()) {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        emitter.moveParticles();
        axis.recompute();
        cb.recompute();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }

    // clean up done automatically thanks to ref_ptr<>
    return 0;
}
