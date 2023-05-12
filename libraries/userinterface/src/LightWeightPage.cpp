#include "userinterface/widgets/LightWeightPage.h"
#include <variant>
#include "utils/Overloading.h"
#include "userinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

LightWeightPage::Info::Info() {
	post_sig = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		std::visit(curan::utils::overloaded{
		[](Empty arg) {

			},
		[](Move arg) {

			},
		[config,page_interaction](Press arg) {
				if(!page_interaction)
					config->stack_page.pop();
			},
		[](Scroll arg) {;

			},
		[](Unpress arg) {

			},
		[](Key arg) {

			},
		[](ItemDropped arg) {;

		} },
		sig);
	};
}

LightWeightPage::LightWeightPage(Info info) : scene{ info.contained }, backgroundcolor{ info.backgroundcolor }, is_dirty{ true }, post_signal_processing{info.post_sig} {

}

std::unique_ptr<LightWeightPage> LightWeightPage::make(Info info) {
	compilation_results results;
	info.contained->linearize_container(results.callable_draw, results.callable_signal);
	std::unique_ptr<LightWeightPage> page = std::unique_ptr<LightWeightPage>(new LightWeightPage{ info });
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
	bool did_interact = false;
	for (auto& sigcall : compiled_scene.callable_signal) {
		if (sigcall(sig, config_draw)) {
			did_interact = true;
			break;
		}
	};
	post_signal_processing(sig, did_interact, config_draw);
	return did_interact;
}

void LightWeightPage::propagate_size_change(SkRect& new_size) {
	if (scene) {
		scene->set_position(new_size);
		scene->framebuffer_resize();
	}
}

}
}