#ifndef CURAN_RUNTIME_EFFECT_HEADER_FILE_
#define CURAN_RUNTIME_EFFECT_HEADER_FILE_

#include <array>
#include <string>
#include "definitions/UIdefinitions.h"

namespace curan {
namespace ui {

class RuntimeEffect{
    using float3 = std::array<float, 3>;

    float time;
    sk_sp<SkRuntimeEffect> effect;

public:
    explicit RuntimeEffect(const std::string& SkSl);

    void draw(SkCanvas* canvas, SkIRect region_to_paint);

};

}
}

#endif