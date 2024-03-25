#include "userinterface/widgets/Overlay.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Page.h"

namespace curan {
namespace ui {

Overlay::Overlay(std::unique_ptr<Container> contained,
                 SkColor in_backgroundcolor,bool tight) : main_page{std::move(LightWeightPage::make(std::move(contained),in_backgroundcolor,tight))}
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

Overlay::Overlay(std::unique_ptr<Container> contained,
                 SkColor in_backgroundcolor,post_signal_callback callback,bool tight) : main_page{std::move(LightWeightPage::make(std::move(contained),in_backgroundcolor,tight))}
{
    main_page->set_post_signal(callback);
}

std::unique_ptr<LightWeightPage> Overlay::take_ownership(){
    compile();
    return std::move(main_page);
}

std::unique_ptr<Overlay> Overlay::make(std::unique_ptr<Container> contained,SkColor backgroundcolor,bool tight){
	std::unique_ptr<Overlay> overlay = std::unique_ptr<Overlay>(new Overlay(std::move(contained),backgroundcolor,tight));
	return overlay;
}

std::unique_ptr<Overlay> Overlay::make(std::unique_ptr<Container> contained,post_signal_callback callback,SkColor backgroundcolor,bool tight){
	std::unique_ptr<Overlay> overlay = std::unique_ptr<Overlay>(new Overlay(std::move(contained),backgroundcolor,callback,tight));
	return overlay;
}

void Overlay::compile(){

}

}
}