#pragma once

#include "../builder/BuilderProps.h"
#include "../utils/additionalFuncs.h"
#include "../utils/vectorUtils.h"
#include "Arrow.h"
#include "Text.h"
#include <vsg/all.h>

namespace vsgps {
class Axis {
  public:
    Axis(BuilderProps &props);

    void addToScene(vsg::ref_ptr<vsg::Group> &scene);

    void fixAxis(bool fc = true, bool fr = false);

    void recompute();

  protected:
    vsg::ref_ptr<vsg::Group> root, xTicksGroup, yTicksGroup, zTicksGroup;
    vsg::ref_ptr<vsg::MatrixTransform> axisCenter;
    BuilderProps props;
    std::array<Arrow, 3> arrows;
    bool fixCenter = true, fixRadius = false;
    vsg::dvec3 center, radius;
    std::vector<Text> ticks;
    std::vector<Text> xticks;
    std::vector<Text> yticks;
    std::vector<Text> zticks;
    Text xLabel, yLabel, zLabel;
    size_t nTicks = 4, maxTicks = 10;

    void setupAxis();
    void setupTicks();
    void recomputeTicks();

    template <size_t i>
    void recomputeAxisTicks(std::vector<Text> &ticksVector,
                            vsg::ref_ptr<vsg::Group> group) {
        vsg::dvec3 starting = center, ending = center, step, pos;
        starting[i] -= radius[i] * .5;
        ending[i] += radius[i] * .5;
        step = ending - starting;
        size_t N =
            staircase(vsg::length(step) * 2.0 * vsg::PI / 4.0, 4) / vsg::PI + 2;
        N += N % 2;
        N = std::clamp(N, 0ul, maxTicks);
        step = (ending - starting) / (double)N;
        std::vector<size_t> ticksIndices;
        ticksIndices.resize(N - 1);
        linspace(ticksIndices, 1UL);
        remove(ticksIndices, (size_t)std::ceil(N * .5));
        ticksIndices.pop_back();
        std::string label;
        size_t xti = 0;
        for (auto &tv : ticksVector) {
            if (xti >= ticksIndices.size())
                break;
            pos = starting + step * (double)ticksIndices[xti];
            label = vsg::make_string(pos[i]);
            label = label.substr(0, label.find_first_not_of('.') + 3 +
                                        (pos[i] < 0));
            tv.changeText(label);
            tv.moveText(pos);
            xti++;
        }
    }
};
} // namespace vsgps
