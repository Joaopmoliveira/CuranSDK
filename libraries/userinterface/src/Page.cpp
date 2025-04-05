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
			bool val = false;
			std::visit([&val](auto&& arg){ val = arg->terminated(); if(val) arg->terminate(false); }, *begin);
			if(val)
				begin = page_stack.erase(begin);
			else
				++begin;
		}
	}
	if (!page_stack.empty()) {
		auto image = canvas->getSurface()->makeImageSnapshot();
		canvas->drawImage(image, 0, 0, options, &bluring_paint);
		std::visit([canvas](auto&& arg){ arg->draw(canvas); }, page_stack.back());
	}
	return *(this);
}

bool Page::propagate_signal(Signal sig, ConfigDraw* config){
	bool val = false;
	if(!page_stack.empty()){
		std::visit([&val,sig,config](auto&& arg){ val = arg->propagate_signal(sig, config); }, page_stack.back());
		return val;
	}
	return main_page->propagate_signal(sig, config);
}

void Page::propagate_heartbeat(ConfigDraw* config){
	Empty data;
	Signal heartbeat = data;
	if(!page_stack.empty()){
		std::visit([heartbeat,config](auto&& arg){ arg->propagate_signal(heartbeat, config); }, page_stack.back());
	} else {
		main_page->propagate_signal(heartbeat, config);
	}
}

Page& Page::propagate_size_change(const SkRect& new_size){
	main_page->propagate_size_change(new_size);
	for (auto& pag : page_stack)
		std::visit([new_size](auto&& arg){ arg->propagate_size_change(new_size); }, pag);
	return *(this);
}

Page& Page::pop(){
	if (!page_stack.empty())
		std::visit([](auto&& arg){ arg->terminate(true);}, page_stack.back());
	return *(this);
}

Page& Page::replace_all(std::unique_ptr<Overlay> overlay){
	std::unique_ptr<LightWeightPage> local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		std::for_each(page_stack.begin(),page_stack.end(),[](pointer_type& page){std::visit([](auto&& arg){ arg->terminate(true);}, page);}); 
		page_stack.emplace_back(std::move(local));
	}
	return *(this);
}

Page& Page::replace_last(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		local->terminate(true);
		page_stack.emplace_back(std::move(local));
	}
	return *(this);
}

Page& Page::stack(std::unique_ptr<Overlay> overlay){
	auto local = overlay->take_ownership();
	local->propagate_size_change(previous_size);
	std::lock_guard<std::mutex> g{mut};
	page_stack.emplace_back(std::move(local));
	return *(this);
}

Page& Page::replace_all(std::shared_ptr<LightWeightPage> local){
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		std::for_each(page_stack.begin(),page_stack.end(),[](pointer_type& page){std::visit([](auto&& arg){ arg->terminate(true);}, page);}); 
		page_stack.emplace_back(local);
	}
	return *(this);
}

Page& Page::replace_last(std::shared_ptr<LightWeightPage> local){
	local->propagate_size_change(previous_size);
	{
		std::lock_guard<std::mutex> g{mut};
		local->terminate(true);
		page_stack.emplace_back(local);
	}
	return *(this);
}

Page& Page::stack(std::shared_ptr<LightWeightPage> local){
	local->propagate_size_change(previous_size);
	std::lock_guard<std::mutex> g{mut};
	page_stack.emplace_back(local);
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

Page& Page::clear_overlays(){
	std::lock_guard<std::mutex> g{mut};
	std::for_each(page_stack.begin(),page_stack.end(),[](pointer_type& page){std::visit([](auto&& arg){ arg->terminate(true);}, page);}); 
	return  *(this);
}

}
}