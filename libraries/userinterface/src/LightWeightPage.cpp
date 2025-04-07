#include "userinterface/widgets/LightWeightPage.h"
#include <iostream>

namespace curan {
namespace ui {

LightWeightPage::LightWeightPage(std::unique_ptr<Container> contained, SkColor backgroundcolor,SkRect computed_minimum_size, bool tight):cached_minimum_size{computed_minimum_size},scene{std::move(contained)},backgroundcolor{backgroundcolor}, is_tight{tight}{
    post_signal_processing = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		return;
	};
	is_dirty = true;
	paint.setStyle(SkPaint::kFill_Style);
	paint.setColor(SkColorSetARGB(75,255,255,255));

}

LightWeightPage::~LightWeightPage(){

}

std::unique_ptr<LightWeightPage> LightWeightPage::make(std::unique_ptr<Container> contained, SkColor backgroundcolor, bool tight){
	compilation_results results;
	contained->compile();
	contained->linearize_container(results.callable_draw, results.callable_signal);
	auto min_size = contained->minimum_size();
	std::unique_ptr<LightWeightPage> page = std::unique_ptr<LightWeightPage>(new LightWeightPage{std::move(contained),backgroundcolor,min_size,tight});
	page->compiled_scene = results;
	return page;
}

LightWeightPage& LightWeightPage::draw(SkCanvas* canvas){
	if(is_tight)
		canvas->drawRoundRect(scene->get_position(),10,10,paint);
		
	canvas->drawColor(backgroundcolor);
	for (auto& drawcall : compiled_scene.callable_draw)
		drawcall(canvas);
    return *(this);
}

bool LightWeightPage::propagate_signal(Signal sig, ConfigDraw* config){
	bool did_interact = false;
	for (auto& sigcall : compiled_scene.callable_signal) {
		if (sigcall(sig, config)) {
			did_interact = true;
			break;
		}
	};
	post_signal_processing(sig, did_interact, config);
	return did_interact;
}

LightWeightPage& LightWeightPage::set_post_signal(post_signal_callback call){
    post_signal_processing = call;
    return *(this);
}

LightWeightPage& LightWeightPage::propagate_size_change(const SkRect& new_size){
	if(scene){
		if(is_tight){
			constexpr size_t default_padding = 100;
			SkRect centered_minimum_position = SkRect::MakeXYWH(new_size.centerX()-cached_minimum_size.width()/2.0f,new_size.centerY()-cached_minimum_size.height()/2.0f,default_padding+cached_minimum_size.width(),default_padding+cached_minimum_size.height());
			if( new_size.width()>centered_minimum_position.width() && new_size.height()>centered_minimum_position.height()){
				scene->set_position(centered_minimum_position);
				scene->framebuffer_resize(new_size);
			} else {
				scene->set_position(new_size);
				scene->framebuffer_resize(new_size);		
			}

		} else {
			scene->set_position(new_size);
			scene->framebuffer_resize(new_size);
		}

	}
    return *(this);
}

}
}