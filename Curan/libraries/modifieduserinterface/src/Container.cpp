#include "modifieduserinterface/widgets/Container.h"

namespace curan {
namespace ui {

Container::Container(ContainerType type, Arrangement arragement){

}

drawablefunction Container::draw(){

}

callablefunction Container::call(){

}

bool Container::is_leaf(){

}

Container& Container::framebuffer_resize(){

}

Container& Container::linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal){

}

}
}