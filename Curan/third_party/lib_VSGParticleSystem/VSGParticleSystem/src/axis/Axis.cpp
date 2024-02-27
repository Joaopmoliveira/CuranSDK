#include <vsgParticleSystem/axis/Axis.h>
#include <vsgParticleSystem/utils/vsgAdditionalOperators.h>
#include <vsgParticleSystem/utils/vsgAdditionalFunctions.h>


namespace vsgps {
Axis::Axis(BuilderProps &props)
    : xLabel{"x", {.01, 0, 0}, props.options},
      yLabel{"y", {0, .01, 0}, props.options},
      zLabel{"z", {0, 0, .01}, props.options} {
    this->props = props;
    root = vsg::Group::create();
    props.root->addChild(root);
    xLabel.addToScene(root);
    yLabel.addToScene(root);
    zLabel.addToScene(root);
    xTicksGroup = vsg::Group::create();
    yTicksGroup = vsg::Group::create();
    zTicksGroup = vsg::Group::create();
    root->addChild(xTicksGroup);
    root->addChild(yTicksGroup);
    root->addChild(zTicksGroup);
    axisCenter = vsg::MatrixTransform::create();
    axisCenter->matrix = vsg::translate(.0, .0, .0);
    root->addChild(axisCenter);

    center = {0, 0, 0};
    radius = {.02, .02, .02};

    for (size_t i = 0; i < 3; i++) {
        arrows[i].makeArrow(props);
        switch (i) {
        case 0:
            arrows[i].recomputeArrow({-.01, 0, 0}, {.02, 0, 0});
            break;
        case 1:
            arrows[i].recomputeArrow({0, -.01, 0}, {0, .02, 0});
            break;
        case 2:
            arrows[i].recomputeArrow({0, 0, -.01}, {0, 0, .02});
            break;
        default:
            break;
        }
        arrows[i].addToScene(root);
    }

    setupTicks();
}

void Axis::fixAxis(bool fc, bool fr) {
    fixCenter = fc;
    fixRadius = fr;
}

void Axis::recompute() {
    if (fixCenter && fixRadius)
        return;
    auto [c, r] = computeCenterRadius(props.root);
    if (!fixCenter)
        center = c;
    if (!fixRadius) {
        radius = r;
    }

    // Axis
    vsg::dvec3 arrowOrigin;
    for (size_t i = 0; i < 3; i++) {
        arrowOrigin = center;
        arrowOrigin[i] -= radius[i] / 2.0;
        switch (i) {
        case 0:
            arrows[i].recomputeArrow(arrowOrigin, {radius.x, 0., 0.});
            break;
        case 1:
            arrows[i].recomputeArrow(arrowOrigin, {0., radius.y, 0.});
            break;
        case 2:
            arrows[i].recomputeArrow(arrowOrigin, {0., 0., radius.z});
            break;
        default:
            break;
        }
    }

    // Ticks
    recomputeAxisTicks<0>(xticks, xTicksGroup);
    recomputeAxisTicks<1>(yticks, yTicksGroup);
    recomputeAxisTicks<2>(zticks, zTicksGroup);

    // Labels
    arrowOrigin = center;
    arrowOrigin[0] += .9 * radius[0] / 2.0;
    xLabel.moveText(arrowOrigin);
    arrowOrigin = center;
    arrowOrigin[1] += .9 * radius[1] / 2.0;
    yLabel.moveText(arrowOrigin);
    arrowOrigin = center;
    arrowOrigin[2] += .9 * radius[2] / 2.0;
    zLabel.moveText(arrowOrigin);
}

void Axis::setupTicks() {
    for (size_t i = 0; i < maxTicks; i++) {
        xticks.push_back(Text{"", {0, 0, 0}, props.options});
        xticks[xticks.size() - 1].addToScene(xTicksGroup);
        yticks.push_back(Text{"", {0, 0, 0}, props.options});
        yticks[yticks.size() - 1].addToScene(yTicksGroup);
        zticks.push_back(Text{"", {0, 0, 0}, props.options});
        zticks[zticks.size() - 1].addToScene(zTicksGroup);
    }
}

void Axis::recomputeTicks() {
    vsg::dvec3 startingPoint, endingPoint, step, pos;
    double norm;
    std::string label;
    size_t N = 6, ticksIdx = 0;
    for (size_t i = 0; i < 3; i++) {
        startingPoint = center;
        startingPoint[i] -= radius[i] * .5;
        endingPoint = center;
        endingPoint[i] += radius[i] * .5;
        norm = vsg::length(endingPoint - startingPoint);
        step = (endingPoint - startingPoint) / (double)N;
        for (size_t j = 1; j < N - 1; j++) {
            if (j == N * .5)
                continue;
            pos = startingPoint + step * j;
            label = vsg::make_string(pos[i]);
            ticks[ticksIdx].changeText(
                label.substr(0, label.find_first_of('.') + 3));
            ticks[ticksIdx].moveText(pos);
            ticksIdx++;
        }
    }
}

void Axis::addToScene(vsg::ref_ptr<vsg::Group> &scene) {
    scene->addChild(root);
}

} // namespace vsgps
