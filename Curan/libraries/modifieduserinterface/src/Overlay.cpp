#include "modifieduserinterface/widgets/Overlay.h"
#include "utils/Overloading.h"
#include "modifieduserinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

Overlay::Overlay(post_signal_callback&& post_sig,
                std::unique_ptr<Container> contained,
                SkColor in_backgroundcolor) : main_page{LightWeightPage::make(std::move(contained),backgroundcolor)}
{
}

std::unique_ptr<LightWeightPage> Overlay::take_ownership(){
    compile();
    return std::move(main_page);
}

void Overlay::compile(){

}

}
}