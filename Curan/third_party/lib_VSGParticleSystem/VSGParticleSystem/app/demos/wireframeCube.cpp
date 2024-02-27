#include <iostream>
#include <vsg/all.h>
#include <vsgParticleSystem/all.h>
#include <vsgXchange/all.h>

inline double rand01() { return (double)std::rand() / (double)RAND_MAX; }

int main(int argc, char const *argv[]) {
    std::srand(std::time(nullptr));

    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Guerrilla Rodrigues";

    auto root = vsg::Group::create();
    auto builder = vsg::ResolutionBuilder::create();
    builder->options = options;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    size_t numOfObjects = 10;
    geomInfo.positions = vsg::vec4Array::create(numOfObjects);
    auto colors = vsg::vec4Array::create(numOfObjects);
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);

    vsg::StateInfo stateInfo;
    stateInfo.wireframe = true;

    std::array<vsgps::Arrow, 3> arrows;

    root->addChild(builder->createBox(geomInfo, stateInfo));

    vsgps::BuilderProps props{builder, options, &geomInfo, stateInfo, root};
    vsgps::Axis axis(props);
    axis.fixAxis(true, false);

    vsg::dvec3 a{rand01(), rand01(), rand01()}, b{rand01(), rand01(), rand01()};
    auto sphereA = vsg::MatrixTransform::create();
    sphereA->matrix = vsg::scale(.1, .1, .1);
    sphereA->matrix = sphereA->transform(vsg::translate(a));
    sphereA->addChild(builder->createSphere(geomInfo, stateInfo));
    root->addChild(sphereA);
    auto sphereB = vsg::MatrixTransform::create();
    sphereB->matrix = vsg::scale(.1, .1, .1);
    sphereB->matrix = sphereB->transform(vsg::translate(b));
    sphereB->addChild(builder->createSphere(geomInfo, stateInfo));
    root->addChild(sphereB);

    vsg::dvec3 b2a = vsg::normalize(b - a);
    double theta = std::acos(vsg::dot({.0, .0, 1.}, b2a));
    vsg::dvec3 k = vsg::normalize(vsg::cross({.0, .0, 1.}, b2a));
    // vsg::info("k = ", k, "\ntheta = ", theta, "\nnorm = ", vsg::length(b -
    // a));
    double ratio = .95;

    auto adjustingCylinder = vsg::MatrixTransform::create();
    adjustingCylinder->matrix = vsg::scale(.02, .02, ratio);
    adjustingCylinder->matrix = adjustingCylinder->transform(
        vsg::translate(.0, .0, -(1.0 - ratio) / 2.0));
    adjustingCylinder->addChild(builder->createCylinder(geomInfo, stateInfo));
    // root->addChild(adjustingCylinder);

    auto adjustingTip = vsg::MatrixTransform::create();
    adjustingTip->matrix = vsg::scale(.07, .07, .1);
    adjustingTip->matrix =
        adjustingTip->transform(vsg::translate(.0, .0, ratio / 2.0));
    adjustingTip->addChild(builder->createCone(geomInfo, stateInfo));
    // root->addChild(adjustingTip);

    auto adjustArrow = vsg::MatrixTransform::create();
    adjustArrow->matrix = vsg::translate(.0, .0, .5);
    adjustArrow->addChild(adjustingCylinder);
    adjustArrow->addChild(adjustingTip);

    auto arrowTransform = vsg::MatrixTransform::create();
    arrowTransform->matrix = vsg::scale(1.0, 1.0, vsg::length(b - a));
    arrowTransform->matrix = arrowTransform->transform(vsg::rotate(theta, k));
    arrowTransform->matrix = arrowTransform->transform(vsg::translate(a));
    arrowTransform->addChild(adjustArrow);
    root->addChild(arrowTransform);

    auto scalingBox = vsg::MatrixTransform::create();
    root->addChild(scalingBox);
    scalingBox->addChild(builder->createBox(geomInfo, stateInfo));
    scalingBox->matrix = vsg::translate(1.0, rand01(), rand01());

    auto moveText = vsg::MatrixTransform::create();
    moveText->matrix = vsg::translate(0., 0., 0.);
    root->addChild(moveText);
    vsgps::Text text{"Move me!", {0., 0., 0.}, options};
    text.addToScene(moveText);

    props.stateInfo.wireframe = false;
    vsgps::Colorbar cb{props};
    props.stateInfo.wireframe = true;

    // #########################################################################
    // #########################################################################
    // #########################################################################
    auto viewer = vsg::Viewer::create();
    vsg::ref_ptr<vsg::Window> window(vsg::Window::create(windowTraits));
    if (!window) {
        std::cout << "Could not create windows." << std::endl;
        return 1;
    }

    viewer->addWindow(window);
    vsg::ComputeBounds computeBounds;
    root->accept(computeBounds);
    vsg::dvec3 center =
        (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius =
        vsg::length(computeBounds.bounds.max - computeBounds.bounds.min);
    double nearFarRatio = 0.0001;

    vsg::dvec3 offset{radius, radius, radius};
    offset *= 1.5;
    vsg::dvec3 up{0, 0, 1};
    auto lookAt = vsg::LookAt::create(center + offset, center, up);

    auto perspective = vsg::Perspective::create(
        60.0,
        static_cast<double>(window->extent2D().width) /
            static_cast<double>(window->extent2D().height),
        nearFarRatio * radius, radius * 100.0);

    auto camera = vsg::Camera::create(
        perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));

    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));

    viewer->compile();

    double t = 0.;
    while (viewer->advanceToNextFrame()) {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        // moveText->matrix = vsg::translate(std::sin(t), std::cos(t), 0.);
        text.moveText({std::sin(t) * (2. + t), std::cos(t) * (2. + t), t});
        text.changeText(vsg::make_string("Move me in ",
                                         viewer->getFrameStamp()->frameCount));

        cb.recompute();
        axis.recompute();
        scalingBox->matrix = vsg::translate(std::sin(t) * 5., 0.0, 0.0);
        t += .002;

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }

    return 0;
}
