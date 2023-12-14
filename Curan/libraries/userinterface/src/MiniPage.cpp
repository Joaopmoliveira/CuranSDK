#include "userinterface/widgets/Minipage.h"

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
    main_page = std::move(LightWeightPage::make(std::move(container),background));
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
        std::lock_guard<std::mutex> g{get_mutex()};
		return main_page->propagate_signal(sig, config);
	};
	return lamb;
}

void MiniPage::framebuffer_resize(const SkRect& new_page_size){
    std::lock_guard<std::mutex> g{get_mutex()};
    main_page->propagate_size_change(new_page_size);
}

}
}