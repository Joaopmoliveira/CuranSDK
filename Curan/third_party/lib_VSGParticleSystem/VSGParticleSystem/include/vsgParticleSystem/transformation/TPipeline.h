#pragma once

#include "../dynamics/Dynamics.h"
#include "../dynamics/GravityField.h"
#include "SecurityRegion.h"
#include "Transformation.h"
#include <memory>
#include <unordered_map>

namespace vsgps {

struct AddOp {
    Dynamics dynamics;
    GravityField gf;
    SecurityRegion sr;
};

struct AddInfo {
  struct {
    State world;
    State disp;
    State force;
    State deformed;
    State normalized;
  } eqPoint;
};

using Transformations = std::unordered_map<std::string, std::unique_ptr<Transformation>>;
using TransformationsCallback = std::function<void(Transformations &, State const &, State &)>;
using PipelineCallback = std::function<State(Transformations &, AddOp &, State const &)>;

class TPipeline {
  public:
    TPipeline() = default;

    template <typename T> void operator<<(T const &transform) {
        std::unique_ptr<Transformation> t = std::make_unique<T>(transform);
        transformations.insert_or_assign(transform.label, std::move(t));
    }

    void operator()(State const &xi, State &dxi);
    State operator()(State const &xi);
    void inv(State const &xi, State &dxi);

    void setForward(TransformationsCallback const &f);
    void setInverse(TransformationsCallback const &i);
    void setPipeline(PipelineCallback const &p);

    void setAdditionalOperations(AddOp const &a);
    AddOp const &getConstAdditionalOperations() const;
    AddOp &getAdditionalOperations();
    void setAdditionalInformation(AddInfo const &ai);
    void setAdditionalInformation(std::string source);
    AddInfo getAdditionalInformation() const;

  protected:
    Transformations transformations;
    TransformationsCallback forward, inverse;
    PipelineCallback pipeline;
    AddOp addOp;
    AddInfo addInfo;
};

TPipeline pipelineFromJson(std::string path);

using Pipeline = TPipeline;

} // namespace vsgps