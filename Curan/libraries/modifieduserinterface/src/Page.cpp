#include "modifieduserinterface/widgets/Page.h"

namespace curan {
namespace ui {

Page::Page(Container&& container,SkColor background) : main_page{[](Signal sig, bool page_interaction, ConfigDraw* config) {return;},std::move(container),background}
{
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
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