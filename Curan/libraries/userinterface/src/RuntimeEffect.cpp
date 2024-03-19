#include "userinterface/widgets/RuntimeEffect.h"

namespace curan {
namespace ui {

RuntimeEffect::RuntimeEffect(const std::string& SkSl){
    auto [l_effect, l_err] = SkRuntimeEffect::MakeForShader(SkString(SkSl.c_str()));
    if(!l_effect)
        throw std::runtime_error(std::string(l_err.c_str()));
    effect = l_effect;
};

void RuntimeEffect::draw(SkCanvas* canvas, SkIRect region_to_paint){
    SkAutoCanvasRestore restore{canvas,false};
    auto width = canvas->getSurface()->width();
    auto height = canvas->getSurface()->height();
    time += 0.016f;
    SkRuntimeShaderBuilder builder{effect};
    builder.uniform("in_time") = time;
    builder.uniform("in_resolution") = float3{ static_cast<float>(width), static_cast<float>(height), 1.0f };
    
    canvas->clipIRect(region_to_paint);
    canvas->drawColor(SK_ColorGRAY);
    SkPaint p;
    p.setShader(builder.makeShader());
    canvas->drawPaint(p);
};

}
}