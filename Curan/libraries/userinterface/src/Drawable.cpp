#include "userinterface/widgets/Drawable.h"

namespace curan {
namespace ui {

Drawable::Drawable(){

}

void Drawable::framebuffer_resize(){
    return ;
}

bool Drawable::is_leaf(){
    return true;
}

SkRect Drawable::minimum_size(){
    return size;
}

}
}
