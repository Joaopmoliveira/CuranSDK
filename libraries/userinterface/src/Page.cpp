#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"

namespace curan {
namespace ui {

Page::Page(Info info) {
	LightWeightPage::Info info_core;
	info_core.backgroundcolor = info.backgroundcolor;
	info_core.contained = info.contained;
	info_core.post_sig = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		return;
	};
	main_page = LightWeightPage::make(info_core);
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
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
	if (!page_stack.empty())
		return page_stack.front()->propagate_signal(sig, config_draw);
	else 
		return main_page->propagate_signal(sig, config_draw);
}

void Page::propagate_size_change(SkRect& new_size) {
	main_page->propagate_size_change(new_size);
	for (auto& pag : page_stack)
		pag->propagate_size_change(new_size);
}

void Page::pop() {
	if (!page_stack.empty())
		page_stack.pop_back();
}

void Page::stack(std::shared_ptr<Overlay> overlay) {
	page_stack.push_back(overlay->take_ownership());
}

}
}