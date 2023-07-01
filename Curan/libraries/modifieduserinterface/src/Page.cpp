#include "modifieduserinterface/widgets/Page.h"

namespace curan {
namespace ui {

explicit Page::Page(Container&& container,SkColor background){

}

Page& Page::draw(SkCanvas* canvas){

}

Page& Page::propagate_signal(Signal sig, ConfigDraw* config){

}

Page& Page::propagate_size_change(SkRect& new_size){

}

Page& Page::pop(){

}

Page& Page::stack(std::shared_ptr<Overlay> overlay){

}

}
}