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
	return *(this);
}

Page& Page::propagate_signal(Signal sig, ConfigDraw* config){
	return *(this);
}

Page& Page::propagate_size_change(SkRect& new_size){
	return *(this);
}

Page& Page::pop(){
	return *(this);
}

Page& Page::stack(Overlay&& overlay){
	return *(this);
}

}
}