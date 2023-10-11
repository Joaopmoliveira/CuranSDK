#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/Window.h"

namespace curan {
namespace ui {

Page::Page(std::unique_ptr<Container> container,SkColor background) : main_page{std::move(LightWeightPage::make(move(container),background))}
{
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
}

Page& Page::draw(SkCanvas* canvas){
	main_page->draw(canvas);
	if (!page_stack.empty()) {
		auto image = canvas->getSurface()->makeImageSnapshot();
		canvas->drawImage(image, 0, 0, options, &bluring_paint);
		page_stack.back()->draw(canvas);
	}
	return *(this);
}

bool Page::propagate_signal(Signal sig, ConfigDraw* config){
	if (!page_stack.empty())
		return page_stack.back()->propagate_signal(sig, config);
	else 
		return main_page->propagate_signal(sig, config);
}

Page& Page::propagate_size_change(const SkRect& new_size){
	main_page->propagate_size_change(new_size);
	for (auto& pag : page_stack)
		pag->propagate_size_change(new_size);
	return *(this);
}

Page& Page::pop(){
	if (!page_stack.empty())
		page_stack.pop_back();
	return *(this);
}

Page& Page::stack(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	page_stack.emplace_back(std::move(local));
	return *(this);
}
 
void Page::update_page(const Window* window){
	previous_size = window->get_size();
	propagate_size_change(previous_size);
}

}
}