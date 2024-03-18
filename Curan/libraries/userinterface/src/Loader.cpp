#include "userinterface/widgets/Loader.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/ImageDisplay.h"

#include <iostream>

namespace curan {
namespace ui {

Loader::Loader(std::unique_ptr<Container> contained,const std::string& in_icon_identifier,IconResources& in_system_icons,ImageDisplay* in_image_display) : 
		main_page{std::move(LightWeightPage::make(std::move(contained),SK_ColorBLACK,false))},
		icon_identifier{in_icon_identifier},
		system_icons{in_system_icons},
		image_display{in_image_display}
{
    	auto post_sig = [inner_image_display = image_display](Signal sig, bool page_interaction, ConfigDraw* config) {
			static size_t index = 0;
			std::visit(utilities::overloaded{
			[&config,inner_image_display](Empty arg) {
				++index;
				if(inner_image_display) inner_image_display->update_custom_drawingcall([](SkCanvas* canvas, SkRect image_rec, SkRect widget_rec){
					SkRegion region{SkIRect::MakeXYWH(widget_rec.x(),widget_rec.y(),widget_rec.width(),widget_rec.height())};
					SkPaint paint;
					paint.setStyle(SkPaint::kFill_Style);
					paint.setAntiAlias(true);
					paint.setStrokeWidth(4);
					paint.setColor(SkColorSetARGB(5*index%0xFF,0x00,0x00,0x00));
					canvas->drawRegion(region,paint);
				});
				if(index>200 && config->stack_page != nullptr){
					config->stack_page->pop();
				}
				},
			[](Move arg) {

				},
			[](Press arg) {
			
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

Loader::~Loader(){
}

std::unique_ptr<LightWeightPage> Loader::take_ownership(){
    compile();
    return std::move(main_page);
}

std::unique_ptr<Loader> Loader::make(const std::string& button_text,IconResources& system_icons){
	auto loc_image_display = ImageDisplay::make();
	auto image_display = loc_image_display.get();
	auto contained = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*contained << std::move(loc_image_display);
	std::unique_ptr<Loader> loader = std::unique_ptr<Loader>(new Loader(std::move(contained),button_text,system_icons,image_display));
	return loader;
}

void Loader::compile(){
	if (system_icons.is_loaded() && icon_identifier.size()>0) {
		auto image = system_icons.get_icon(icon_identifier);
		if(image) image_display->update_image(*image);
	}
	compiled = true;
}

}
}