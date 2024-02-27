#include "vsgParticleSystem/transformation/Normalization.h"
#include <iostream>
#include <vsg/all.h>
#include <vsgParticleSystem/all.h>
#include <vsgXchange/all.h>

double computeRadius(std::vector<vsg::dvec3> const &data,
                     vsg::dvec3 const &center) {
    double radius = 0, tempRadius;
    vsg::dvec3 r;
    for (auto const &d : data) {
        r = d - center;
        // r.z = 0;
        tempRadius = vsg::length(r);
        if (tempRadius > radius)
            radius = tempRadius;
    }
    return radius;
}

int main(int argc, char **argv) {
    std::srand(std::time(nullptr));

    // set up defaults and read command line arguments to override them
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Force Limit Demo";

    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    if (!arguments.read({"--fullscreen", "--fs"}))
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

    // Particle emitter
    vsgps::Emitter emitter(props);

    vsg::Path defaultPath{"../../data/dynamics/force-limit-demo-3D.json"};
    vsg::Path modelPath = arguments.value<vsg::Path>(defaultPath, "--dyn");

    // Starts in displacement coordinates and converts them into force
    // coordinates
    vsgps::Disp2Force disp2Force = vsgps::Disp2Force::fromJson(modelPath);

    // Normalize force coordinates to the model
    auto normalization = vsgps::Normalization::fromJson(modelPath);

    // Deforms space to the same space of the model's dynamic
    vsgps::SoftPlus softplus = vsgps::SoftPlus::fromJson(modelPath);

    // Converts a point in deformed space into its respective velocity vector
    vsgps::Dynamics dynamics = vsgps::Dynamics::fromJson(modelPath);

    // In case there is a invalid state, it is applied a gravity field onto the
    // state until it reaches a valid region
    vsgps::GravityField gravity{2, 0.25, -1., 1.0, 1.0, 3};
    gravity.fromJson(modelPath);

    std::ofstream logFile("../../force-limit-second.log");
    logFile << "i,t,x,y,z,dx,dy,dz\n";
    int index = 0;
    size_t numOfParticles = arguments.value<size_t>(10, "-n");
    double t = 0;
    emitter.setDynamicFunction([&](vsgps::State const &xi, vsgps::State &dxi) {
        vsgps::State currentXi = xi;
        index %= numOfParticles;
        logFile << index << "," << t << "," << xi[0] << "," << xi[1] << ","
                << xi[2] << ",";

        // F = K * delta
        disp2Force(currentXi);
        vsgps::State point4Gravity = currentXi;

        // xi = F / max(abs(F))
        normalization(currentXi);

        // xitilde = ln(- 1 + e ^ (k * xi)) / k
        softplus(currentXi);

        // dxitilde = f(xitilde)
        dxi = dynamics(currentXi);

        // dxi = J ^ (-1) * dxitilde
        softplus.invJ(currentXi, dxi);

        // dF = dxi * max(abs(F))
        normalization.inv(dxi);

        // dF_g = dF + g(F)
        dxi += gravity(point4Gravity, dxi);

        // ddelta = dF_g / K
        disp2Force.inv(dxi);

        logFile << dxi[0] << "," << dxi[1] << "," << dxi[2] << "\n";
        index++;
    });

    std::vector<vsg::dvec3> particlePositions;
    addData(modelPath, props, particlePositions, .1);

    double elevation = arguments.value<double>(0, "-z");
    vsg::dvec3 centerInit = vsg::dvec3(0, 0, elevation);
    double radiusInit = computeRadius(particlePositions, centerInit);

    vsgps::RandomParticleInitializer randomParticleInitializer{
        vsgps::SpatialRange{-1, 1}, vsgps::SpatialRange{-1, 1},
        vsgps::SpatialRange{-1, 1}};
    emitter.populate(numOfParticles, randomParticleInitializer);

    emitter.setStates(dynamics.getVelocityIndexes(), dynamics.getDimension());
    emitter.setNumberOfDiameters(.5);

    vsgps::Axis axis{props};
    axis.fixAxis(true, false);

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

    auto commandGraph = vsg::createCommandGraphForView(window, camera, basis);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));

    // compile all the the Vulkan objects and transfer data required to render
    // the scene
    viewer->compile();

    // rendering main loop
    emitter.tick();
    auto checkTimePoint = std::chrono::steady_clock::now();
    while (viewer->advanceToNextFrame()) {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        emitter.moveParticles();
        axis.recompute();
        cb.recompute();
        t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          checkTimePoint)
                .count();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }

    logFile.close();

    // clean up done automatically thanks to ref_ptr<>
    return 0;
}
