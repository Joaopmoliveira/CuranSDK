#pragma once

#include <vsg/all.h>

namespace vsg {
class VSG_DECLSPEC ResolutionBuilder
    : public Inherit<Builder, ResolutionBuilder> {
  public:
    ref_ptr<Node> createSphere(const GeometryInfo &info = {},
                               const StateInfo &stateInfo = {},
                               const size_t num_columns = 20,
                               const size_t num_rows = 10);
};
VSG_type_name(vsg::ResolutionBuilder);
} // namespace vsg