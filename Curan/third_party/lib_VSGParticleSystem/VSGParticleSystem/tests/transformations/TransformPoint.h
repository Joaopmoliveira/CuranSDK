#pragma once

#include <vector>

struct TransformPoint {
    double xiWorld[3], xiDisp[3], xiForce[3], xiDeformed[3], xiNormalized[3],
        dxiNormalized[3], dxiDeformed[3], dxiForce[3], dxiDisp[3], dxiWorld[3];
};

std::vector<TransformPoint> getTransformData();

