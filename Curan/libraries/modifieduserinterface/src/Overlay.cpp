#include "modifieduserinterface/widgets/Overlay.h"
#include "utils/Overloading.h"
#include "modifieduserinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

Overlay::Overlay(std::unique_ptr<Container> contained,
                 SkColor in_backgroundcolor) : main_page{std::move(LightWeightPage::make(std::move(contained),backgroundcolor))}
{
    	auto post_sig = [](Signal sig, bool page_interaction, ConfigDraw* config) {
		std::visit(utilities::overloaded{
		[](Empty arg) {

			},
		[](Move arg) {

			},
		[config,page_interaction](Press arg) {
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
    main_page->set_post_signal(post_sig);
}

std::unique_ptr<LightWeightPage> Overlay::take_ownership(){
    compile();
    return std::move(main_page);
}

void Overlay::compile(){

}

}
}