#pragma once

#include <vsgParticleSystem/all.h>

class LfD2LBR {
  public:
    LfD2LBR() = default;
    void setPipeline(std::string const &path);
    vsgps::Pipeline const &getPipeline() const;

  protected:
    vsgps::State applyDynamics(vsgps::State const &xi);
    vsgps::Pipeline pipeline;
};
