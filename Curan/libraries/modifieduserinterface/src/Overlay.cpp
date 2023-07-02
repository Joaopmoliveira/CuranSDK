#include "modifieduserinterface/widgets/Overlay.h"
#include "utils/Overloading.h"
#include "modifieduserinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

Overlay::Overlay(post_signal_callback post_sig,
                Container&& contained,
                SkColor backgroundcolor) : main_page{[](Signal sig, bool page_interaction, ConfigDraw* config) {
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

		                                                    } },sig);},std::move(contained),backgroundcolor}
{
}

LightWeightPage&& Overlay::take_ownership(){
    return std::move(main_page);
}

}
}