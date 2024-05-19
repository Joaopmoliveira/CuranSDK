#include "userinterface/widgets/MiniPage.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

MiniPage::MiniPage(std::unique_ptr<Container> container,SkColor background): main_page{std::move(LightWeightPage::make(std::move(container),background))}
{
	imgfilter = SkImageFilters::Blur(10, 10, nullptr);
	bluring_paint.setImageFilter(imgfilter);
	options = SkSamplingOptions();
}

std::unique_ptr<MiniPage> MiniPage::make(std::unique_ptr<Container> container,SkColor background){
	std::unique_ptr<MiniPage> minipage = std::unique_ptr<MiniPage>(new MiniPage{std::move(container),background});
	return minipage;
}

void MiniPage::compile(){
    
}

void MiniPage::construct(std::unique_ptr<Container> container,SkColor background){
    std::lock_guard<std::mutex> g{get_mutex()};
    replacement_main_page = std::move(LightWeightPage::make(std::move(container),background));
    replacement_main_page->propagate_size_change(get_position());
}

MiniPage::~MiniPage(){

}

drawablefunction MiniPage::draw(){
    auto lamb = [this](SkCanvas* canvas) {
        std::lock_guard<std::mutex> g{get_mutex()};
        main_page->draw(canvas);
    };
    return lamb;
}

callablefunction MiniPage::call(){
    auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utilities::overloaded{
			[this,&config](Empty arg) {

			},
			[this,&config](Move arg) {
				for(const auto& localcall : callbacks_move)
            		localcall(this,arg,config);		
			},
			[this,&config](Press arg) {
				for(const auto& localcall : callbacks_press)
            		localcall(this,arg,config);		
			},
			[this,&config](Scroll arg) {;
				for(const auto& localcall : callbacks_scroll)
            		localcall(this,arg,config);		
			},
			[this,&config](Unpress arg) {
				for(const auto& localcall : callbacks_unpress)
            		localcall(this,arg,config);		
			},
			[this,&config](Key arg) {
				for(const auto& localcall : callbacks_key)
            		localcall(this,arg,config);		
			},
			[this,&config](ItemDropped arg) {;
				for(const auto& localcall : callbacks_itemdropped)
            		localcall(this,arg,config);		
			}},
			sig);
		interacted = main_page->propagate_signal(sig, config);
        std::lock_guard<std::mutex> g{get_mutex()};
		if(replacement_main_page){
			main_page = std::move(replacement_main_page);
			replacement_main_page = nullptr;
		}
		return interacted;
	};
	return lamb;
}

void MiniPage::framebuffer_resize(const SkRect& new_page_size){
    std::lock_guard<std::mutex> g{get_mutex()};
    main_page->propagate_size_change(get_position());
}

}
}