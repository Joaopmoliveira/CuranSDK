#include "modifieduserinterface/widgets/Page.h"
#include "modifieduserinterface/widgets/Overlay.h"

namespace curan {
namespace ui {

Page::Page(Container&& container,SkColor background) : main_page{[](Signal sig, bool page_interaction, ConfigDraw* config) {return;},std::move(container),background}
{
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
}

Page& Page::draw(SkCanvas* canvas){
	main_page.draw(canvas);
	if (!page_stack.empty()) {
		auto image = canvas->getSurface()->makeImageSnapshot();
		canvas->drawImage(image, 0, 0, options, &bluring_paint);
		page_stack.back().draw(canvas);
	}
	return *(this);
}

bool Page::propagate_signal(Signal sig, ConfigDraw* config){
	if (!page_stack.empty())
		return page_stack.back().propagate_signal(sig, config);
	else 
		return main_page.propagate_signal(sig, config);
}

Page& Page::propagate_size_change(SkRect& new_size){
	main_page.propagate_size_change(new_size);
	for (auto& pag : page_stack)
		pag.propagate_size_change(new_size);
	return *(this);
}

Page& Page::pop(){
	if (!page_stack.empty())
		page_stack.pop_back();
	return *(this);
}

Page& Page::stack(Overlay&& overlay){
	page_stack.emplace_back(overlay.take_ownership());
	return *(this);
}

}
}