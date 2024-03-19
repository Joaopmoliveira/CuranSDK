#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Loader.h"
#include "userinterface/Window.h"

#include <iostream>

namespace curan {
namespace ui {

Page::Page(std::unique_ptr<Container> container,SkColor background) : main_page{std::move(LightWeightPage::make(std::move(container),background))}
{
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
}

Page& Page::draw(SkCanvas* canvas){
	main_page->draw(canvas);
	{ 
		/*
		This machinery allows us to request overlays to be 
		removed from the stack of overlays on the page
		while being thread safe. We transverse all the nodes
		in the list and remove the ones where a previous request
		deletion of the overlay was requested.
		*/
		std::lock_guard<std::mutex> g{mut};
		for(auto begin = page_stack.begin(); begin!=page_stack.end(); ){
			if((*begin)->terminated())
				begin = page_stack.erase(begin);
			else
				++begin;
		}
	}
	if (!page_stack.empty()) {
		auto image = canvas->getSurface()->makeImageSnapshot();
		canvas->drawImage(image, 0, 0, options, &bluring_paint);
		page_stack.back()->draw(canvas);
	}
	return *(this);
}

bool Page::propagate_signal(Signal sig, ConfigDraw* config){
	return (!page_stack.empty()) ? 
							page_stack.back()->propagate_signal(sig, config) : 
							main_page->propagate_signal(sig, config);
}

void Page::propagate_heartbeat(ConfigDraw* config){
	Empty data;
	Signal heartbeat = data;
	(!page_stack.empty()) ? page_stack.back()->propagate_signal(heartbeat, config) : main_page->propagate_signal(heartbeat, config);
}

Page& Page::propagate_size_change(const SkRect& new_size){
	main_page->propagate_size_change(new_size);
	for (auto& pag : page_stack)
		pag->propagate_size_change(new_size);
	return *(this);
}

Page& Page::pop(){
	if (!page_stack.empty())
		page_stack.back()->terminate(true);
	return *(this);
}

Page& Page::replace_all(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		std::for_each(page_stack.begin(),page_stack.end(),[](std::unique_ptr<curan::ui::LightWeightPage>& page){page->terminate(true);});
		page_stack.emplace_back(std::move(local));
	}
	return *(this);
}

Page& Page::replace_last(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		page_stack.back()->terminate(true);
		page_stack.emplace_back(std::move(local));
	}
	return *(this);
}

Page& Page::clear_overlays(){
	std::lock_guard<std::mutex> g{mut};
	std::for_each(page_stack.begin(),page_stack.end(),[](std::unique_ptr<curan::ui::LightWeightPage>& page){page->terminate(true);});
	return  *(this);
}

Page& Page::stack(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	std::lock_guard<std::mutex> g{mut};
	page_stack.emplace_back(std::move(local));
	return *(this);
}

Page& Page::stack(std::unique_ptr<Loader> loader){
	auto local = loader->take_ownership();
	local->propagate_size_change(previous_size);
	std::lock_guard<std::mutex> g{mut};
	page_stack.emplace_back(std::move(local));
	return *(this);
}
 
void Page::update_page(const Window* window){
	previous_size = window->get_size();
	propagate_size_change(previous_size);
}

}
}