#include "userinterface/widgets/Page.h"

namespace curan {
namespace ui {

Page::Page(std::shared_ptr<Container> contained) : scene{ contained } {

}

std::shared_ptr<Page> Page::make(std::shared_ptr<Container> drawables) {
	compilation_results results;
	drawables->linearize_container(results.callable_draw,results.callable_signal);
	std::shared_ptr<Page> page = std::shared_ptr<Page>(new Page{ drawables });
	page->compiled_scene = results;
	return page;
}

void Page::draw(SkCanvas* canvas) {
	if (is_dirty)
		for (auto& drawcall : compiled_scene.callable_draw)
			drawcall(canvas);
}

void Page::propagate_signal(Signal sig) {
	for (auto& sigcall : compiled_scene.callable_signal) {
		if (sigcall(sig)) {
			is_dirty = true;
			break;
		}
	};
}

}
}