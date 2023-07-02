#include "modifieduserinterface/widgets/Overlay.h"
#include "utils/Overloading.h"
#include "modifieduserinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

Overlay::Overlay(post_signal_callback post_sig,
                Container&& contained,
                SkColor backgroundcolor) : main_page{std::move(contained),backgroundcolor}
{
}

LightWeightPage&& Overlay::take_ownership(){
    return std::move(main_page);
}

}
}