#include "modifieduserinterface/widgets/LightWeightPage.h"

namespace curan {
namespace ui {

LightWeightPage::LightWeightPage(Container&& contained, SkColor backgroundcolor): scene{std::move(contained)},backgroundcolor{backgroundcolor}{
    post_signal_processing = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		return;
	};
}

LightWeightPage::LightWeightPage(const LightWeightPage& other): scene{other.scene}{

}

LightWeightPage& LightWeightPage::operator=(const LightWeightPage& other){
    return *(this);
}

LightWeightPage::LightWeightPage(LightWeightPage&& other): scene{std::move(other.scene)}{

}

LightWeightPage::~LightWeightPage(){

}

LightWeightPage& LightWeightPage::draw(SkCanvas* canvas){
	if (is_dirty) {
		canvas->drawColor(backgroundcolor);
		for (auto& drawcall : compiled_scene.callable_draw)
			drawcall(canvas);
	}
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

LightWeightPage& LightWeightPage::propagate_size_change(SkRect& new_size){
	if (scene) {
		scene.set_position(new_size);
		scene.framebuffer_resize();
	}
    return *(this);
}

}
}