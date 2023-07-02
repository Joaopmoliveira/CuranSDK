#include "modifieduserinterface/widgets/LightWeightPage.h"

namespace curan {
namespace ui {

LightWeightPage::LightWeightPage(post_signal_callback post_sig,Container&& contained, SkColor backgroundcolor): scene{std::move(contained)}{

}

LightWeightPage::LightWeightPage(Container&& contained, SkColor backgroundcolor): scene{std::move(contained)}{

}

LightWeightPage::LightWeightPage(const LightWeightPage& other): scene{other.scene}{

}

LightWeightPage& LightWeightPage::operator=(const LightWeightPage& other){
    return *(this);
}

LightWeightPage::LightWeightPage(LightWeightPage&& other): scene{std::move(other.scene)}{

}

LightWeightPage::~LightWeightPage(){

}

LightWeightPage& LightWeightPage::draw(SkCanvas* canvas){
    return *(this);
}

bool LightWeightPage::propagate_signal(Signal sig, ConfigDraw* config){
    return true;
}

LightWeightPage& LightWeightPage::propagate_size_change(SkRect& new_size){
    return *(this);
}

}
}