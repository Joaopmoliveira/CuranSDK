#include "modifieduserinterface/widgets/LightWeightPage.h"

namespace curan {
namespace ui {

LightWeightPage::LightWeightPage(post_signal_callback post_sig,Container&& contained, SkColor backgroundcolor): scene{std::move(contained)}{

}

LightWeightPage& LightWeightPage::draw(SkCanvas* canvas){

}

bool LightWeightPage::propagate_signal(Signal sig, ConfigDraw* config){

}

LightWeightPage& LightWeightPage::propagate_size_change(SkRect& new_size){

}

}
}