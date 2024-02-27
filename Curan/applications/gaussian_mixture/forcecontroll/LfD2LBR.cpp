#include "LfD2LBR.h"

void LfD2LBR::setPipeline(std::string const &path) {
    pipeline = vsgps::pipelineFromJson(path);
}

vsgps::State LfD2LBR::applyDynamics(vsgps::State const &xi) {
    return pipeline(xi);
}

vsgps::Pipeline const &LfD2LBR::getPipeline() const { return pipeline; }
