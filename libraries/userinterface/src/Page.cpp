#include "userinterface/widgets/Page.h"

namespace curan {
namespace ui {

Page::Page(Info info) {
	LightWeightPage::Info info_core;
	info_core.backgroundcolor = info.backgroundcolor;
	info_core.contained = info.contained;
	info_core.post_sig = [](Signal sig) {
	};
	main_page = LightWeightPage::make(info_core);
}

std::shared_ptr<Page> Page::make(Info info) {
	std::shared_ptr<Page> page = std::shared_ptr<Page>(new Page{ info });
	return page;
}

void Page::draw(SkCanvas* canvas) {
	main_page->draw(canvas);
	if (!page_stack.empty()) {
		auto image = canvas->getSurface()->makeImageSnapshot();
		canvas->drawImage(image, 0, 0, options, &bluring_paint);
		page_stack.front()->draw(canvas);
	}
}

bool Page::propagate_signal(Signal sig, ConfigDraw* config_draw) {
	bool local = false;
	for (auto& sigcall : compiled_scene.callable_signal) {
		if (sigcall(sig, config_draw)) {
			local = true;
			break;
		}
	};
	return local;
}

void Page::propagate_size_change(SkRect& new_size) {
	main_page->propagate_size_change(new_size);
	for (auto& pag : page_stack)
		pag->propagate_size_change(new_size);
}

}
}