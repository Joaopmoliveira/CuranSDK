#include <iostream>
#include <vsg/all.h>
#include <vsg/maths/transform.h>
#include <vsgXchange/all.h>

inline vsg::dvec3 computeK(double alpha) {
    return {0, std::cos(alpha), std::sin(alpha)};
}

struct DiscRecord : public vsg::Inherit<vsg::Visitor, DiscRecord> {
    double theta, alpha;
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    DiscRecord(double t, double a, vsg::ref_ptr<vsg::MatrixTransform> transf) {
        theta = t;
        alpha = a;
        transform = transf;
    }

    void apply(vsg::KeyPressEvent &keyPress) override {
        std::cout << keyPress.keyBase << std::endl;
        switch (keyPress.keyBase) {
        case 113:
            theta -= .1;
            transform->matrix = vsg::rotate(theta, computeK(alpha));
            break;

        case 119:
            theta += .1;
            transform->matrix = vsg::rotate(theta, computeK(alpha));
            break;

        case 101:
            alpha -= .1;
            break;

        case 114:
            alpha += .1;
            break;

        default:
            break;
        }
    }
};

int main(int argc, char const *argv[]) {
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Guerrilla Rodrigues";

    auto root = vsg::Group::create();
    auto builder = vsg::Builder::create();
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

    // todo: add pipe
    auto rotatingAxis = vsg::MatrixTransform::create();
    double theta = 0, alpha = 0;
    rotatingAxis->matrix = vsg::rotate(theta, computeK(alpha));
    root->addChild(rotatingAxis);

    auto pipeing = vsg::MatrixTransform::create();
    pipeing->matrix = vsg::scale(.5, .5, 4.0);
    rotatingAxis->addChild(pipeing);
    pipeing->addChild(builder->createCylinder(geomInfo, stateInfo));

    // todo: add reference cube
    auto refCubeTranslate = vsg::MatrixTransform::create();
    refCubeTranslate->matrix = vsg::translate(0.0, 2.0, -2.0);
    refCubeTranslate->addChild(builder->createBox(geomInfo, stateInfo));
    root->addChild(refCubeTranslate);

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
        30.0,
        static_cast<double>(window->extent2D().width) /
            static_cast<double>(window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    auto camera = vsg::Camera::create(
        perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    // viewer->addEventHandler(vsg::Trackball::create(camera));
    viewer->addEventHandler(DiscRecord::create(theta, alpha, rotatingAxis));

    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));

    viewer->compile();

    while (viewer->advanceToNextFrame()) {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }

    return 0;
}
