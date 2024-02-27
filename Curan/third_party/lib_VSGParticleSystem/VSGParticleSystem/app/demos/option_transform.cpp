#include <iostream>
#include <vsg/all.h>
#include <vsgParticleSystem/all.h>
#include <vsgXchange/all.h>

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
        "../../data/dynamics/2023_10_04_demo-3D.json", "--dyn");
    auto dynamics = vsgps::Dynamics::fromJson(dynFile);

    std::vector<vsg::dvec3> particlePositions;
    vsgps::State eqPoint(dynamics.getEquilibriumPoint());
    double threshold = 1e-5;
    std::vector<std::string> modes{"world", "disp", "force", "deformed",
                                   "normalized"};
    auto modeChoice = arguments.value<std::string>("disp", {"-m", "--mode"});
    
    auto w2d = vsgps::World2Disp::fromJson(dynFile);
    auto d2f = vsgps::Disp2Force::fromJson(dynFile);
    auto sp = vsgps::SoftPlus::fromJson(dynFile);
    auto norm = vsgps::Normalization::fromJson(dynFile);
    auto g = vsgps::GravityField::fromJson(dynFile);
    
    vsgps::State xiForce(dynamics.getDimension());
    vsgps::State xiDeformed(dynamics.getDimension());
    vsgps::State xiNorm(dynamics.getDimension());

    switch (std::find(modes.begin(), modes.end(), modeChoice) - modes.begin()) {
    case 0: {
        std::cout << "Chooses world\n";
        vsgps::addDataWithTransform(
            dynFile, props, particlePositions,
            [&](vsgps::State const &state, vsg::dvec3 &pos) {
                xiForce.setZero();
                norm.inv(state, xiForce);
                sp.inv(xiForce);
                d2f.inv(xiForce);
                w2d.inv(xiForce);
                for (size_t i = 0; i < 3; i++) {
                    pos[i] = xiForce(i);
                }
            });
        emitter.setDynamicFunction(
            [&](vsgps::State const &xi, vsgps::State &dxi) {
                w2d.setIsPosition(true);
                w2d(xi, xiDeformed);
                d2f(xiDeformed, xiForce);
                sp(xiForce);
                norm(xiForce, xiNorm);
                dxi = dynamics(xiNorm);
                norm.inv(dxi);
                sp.invJ(xiForce, dxi);
                d2f.inv(dxi);
                dxi = g(xiDeformed, dxi);
                w2d.setIsPosition(false);
                w2d.inv(dxi);
            });
        break;
    }
    case 1: {
        std::cout << "Chooses disp\n";
        vsgps::addDataWithTransform(
            dynFile, props, particlePositions,
            [&](vsgps::State const &state, vsg::dvec3 &pos) {
                xiForce.setZero();
                norm.inv(state, xiForce);
                sp.inv(xiForce);
                d2f.inv(xiForce);
                for (size_t i = 0; i < 3; i++) {
                    pos[i] = xiForce(i);
                }
            },
            .01);
        emitter.setDynamicFunction(
            [&](vsgps::State const &xi, vsgps::State &dxi) {
                // vsg::info("xiDisp =\n", xi);
                d2f(xi, xiForce);
                // vsg::info("xiForce =\n", xiForce);
                sp(xiForce);
                // vsg::info("xitilde =\n", xiForce);
                norm(xiForce, xiNorm);
                // vsg::info("xiNormalized =\n", xiNorm);
                dxi = dynamics(xiNorm);
                // vsg::info("dxiNormalized =\n", dxi);
                norm.inv(dxi);
                // vsg::info("dxitilde =\n", dxi);
                sp.invJ(xiForce, dxi);
                // vsg::info("dxiForce =\n", dxi);
                d2f.inv(dxi);
                // vsg::info("dxiDisp =\n", dxi);
                dxi = g(xi, dxi);
                // vsg::info("dxiGravity =\n", dxi);
                // vsg::info("\n");
            });
        break;
    }
    case 2: {
        std::cout << "Chooses force\n";
        vsgps::addDataWithTransform(
            dynFile, props, particlePositions,
            [&](vsgps::State const &state, vsg::dvec3 &pos) {
                xiForce.setZero();
                norm.inv(state, xiForce);
                sp.inv(xiForce);
                for (size_t i = 0; i < 3; i++) {
                    pos[i] = xiForce(i);
                }
            });
        emitter.setDynamicFunction(
            [&](vsgps::State const &xi, vsgps::State &dxi) {
                xiDeformed.setZero();
                sp(xi, xiDeformed);
                norm(xiDeformed, xiNorm);
                dxi = dynamics(xiNorm);
                norm.inv(dxi);
                sp.invJ(xiDeformed, dxi);
            });
        break;
    }
    case 3: {
        std::cout << "Chooses deformed\n";
        vsgps::addDataWithTransform(
            dynFile, props, particlePositions,
            [&](vsgps::State const &state, vsg::dvec3 &pos) {
                xiForce.setZero();
                norm.inv(state, xiForce);
                for (size_t i = 0; i < 3; i++) {
                    pos[i] = xiForce(i);
                }
            });
        emitter.setDynamicFunction(
            [&](vsgps::State const &xi, vsgps::State &dxi) {
                xiNorm.setZero();
                norm(xi, xiNorm);
                dxi = dynamics(xiNorm);
                norm.inv(dxi);
            });
        break;
    }
    case 4: {
        std::cout << "Chooses normalized\n";
        vsgps::addData(dynFile, props, particlePositions);
        emitter.setDynamicFunction(
            [&](vsgps::State const &xi, vsgps::State &dxi) {
                dxi = ((xi - eqPoint).norm() < 1e-5)
                          ? vsgps::State::Zero(dxi.size())
                          : dynamics(xi);
            });
        break;
    }
    case 5:
    default: {
        std::string showChoice =
            modeChoice.compare("") == 0 ? "empty" : modeChoice;
        std::cout << "Mode incorrect (" << showChoice
                  << "), please chose one of the following:\n";
        for (auto const &m : modes) {
            std::cout << " - " << m << "\n";
        }
        return 1;
    }
    }

    double elevation = arguments.value<double>(-1, "-z");
    size_t numOfParticles = arguments.value<size_t>(5, "-n");
    vsg::dvec3 centerInit =
        vsg::dvec3(eqPoint(0), eqPoint(1), eqPoint(2) + elevation);
    double radiusInit = vsgps::computeRadius(particlePositions, centerInit);
    vsg::info("radiusInit = ", radiusInit, "\ninfo: ", centerInit);

    vsgps::ShapeParticleInitializer spi{radiusInit, numOfParticles, centerInit};
    // spi.setElevation(spi.getElevation() * arguments.read({"-v",
    // "--vertical"}));
    emitter.populate(numOfParticles, spi, radiusInit * .05);
    emitter.setStates(dynamics.getVelocityIndexes(), dynamics.getDimension());
    emitter.setNumberOfDiameters(.5);

    vsgps::Axis axis{props};
    axis.fixAxis(true, false);

    vsgps::Colorbar cb{props};

    // #########################################################################
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

    if (std::isinf(radius)) {
        std::runtime_error("Radius was infinite, couldn't create scene!");
        return 1;
    }

    // todo: build a global function that generates a transformation pipeline
    // from a JSON's path

    // set up the camera
    vsg::dvec3 offset{radius * 1.2, radius * 1.2, radius * .5 * 1.2};
    vsg::dvec3 up{0, 0, 1};
    auto lookAt = vsg::LookAt::create(center + offset, center, up);

    vsg::ref_ptr<vsg::Perspective> perspective = vsg::Perspective::create(
        60.0,
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
