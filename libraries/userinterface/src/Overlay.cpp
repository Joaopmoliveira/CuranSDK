#include "userinterface/widgets/Overlay.h"
#include <variant>
#include "utils/Overloading.h"
#include "userinterface/widgets/ConfigDraw.h"
#include <iostream>

namespace curan {
namespace ui {

Overlay::Info::Info(){
	post_sig = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		std::visit(curan::utils::overloaded{
		[](Empty arg) {

			},
		[](Move arg) {

			},
		[config,page_interaction](Press arg) {
				if(page_interaction)
					std::cout << "Pressed\nPage interaction: true \n";
				else
					std::cout << "Pressed\nPage interaction: false \n";
				if (!page_interaction && config->stack_page != nullptr)
					config->stack_page->pop();
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

Overlay::Overlay(Info info) {
	LightWeightPage::Info info_core;
	info_core.backgroundcolor = info.backgroundcolor;
	info_core.contained = info.contained;
	info_core.post_sig = info.post_sig;
	main_page = LightWeightPage::make(info_core);
}

std::shared_ptr<Overlay> Overlay::make(Info info) {
	std::shared_ptr<Overlay> page = std::shared_ptr<Overlay>(new Overlay{ info });
	return page;
}

std::unique_ptr<LightWeightPage> Overlay::take_ownership() {
	return std::move(main_page);
}


}
}