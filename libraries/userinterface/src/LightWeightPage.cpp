#include "userinterface/widgets/LightWeightPage.h"

namespace curan {
namespace ui {

LightWeightPage::LightWeightPage(Info info) : scene{ info.contained }, backgroundcolor{ info.backgroundcolor }, is_dirty{ true } {

}

std::shared_ptr<LightWeightPage> LightWeightPage::make(Info info) {
	compilation_results results;
	info.contained->linearize_container(results.callable_draw, results.callable_signal);
	std::shared_ptr<LightWeightPage> page = std::shared_ptr<LightWeightPage>(new LightWeightPage{ info });
	page->compiled_scene = results;
	return page;
}

void LightWeightPage::draw(SkCanvas* canvas) {
	if (is_dirty) {
		canvas->drawColor(backgroundcolor);
		for (auto& drawcall : compiled_scene.callable_draw)
			drawcall(canvas);
	}
}

bool LightWeightPage::propagate_signal(Signal sig, ConfigDraw* config_draw) {
	bool local = false;
	for (auto& sigcall : compiled_scene.callable_signal) {
		if (sigcall(sig, config_draw)) {
			local = true;
			break;
		}
	};
	return local;
}

void LightWeightPage::propagate_size_change(SkRect& new_size) {
	if (scene) {
		scene->set_position(new_size);
		scene->framebuffer_resize();
	}
}

}
}