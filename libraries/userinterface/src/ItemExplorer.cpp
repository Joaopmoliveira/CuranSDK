#include "userinterface/widgets/ItemExplorer.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "utils/Overloading.h"
#include <iostream>

namespace curan {
namespace ui {

Item::Item(size_t in_identifier, std::string in_text): identifier{in_identifier},text{in_text}{

}

std::unique_ptr<ItemExplorer> ItemExplorer::make(const std::string& default_icon_name,IconResources& system_icons,bool exclusive){
	return std::unique_ptr<ItemExplorer>(new ItemExplorer(default_icon_name,system_icons,exclusive));
}

ItemExplorer::ItemExplorer(const std::string& default_icon_name,IconResources& system_icons,bool exclusive):is_exclusive{exclusive}{
			color_background = SK_ColorBLACK;
			color_hover = SK_ColorLTGRAY;
			color_selected = SkColorSetARGB(255, 77, 195, 255);
			color_waiting = SK_ColorGRAY;
			color_text = SK_ColorWHITE;

			typeface = defaultTypeface();

			paint_image_background.setStyle(SkPaint::kFill_Style);
			paint_image_background.setAntiAlias(true);
			paint_image_background.setStrokeWidth(4);
			paint_image_background.setColor(color_waiting);

			paint_background.setStyle(SkPaint::kFill_Style);
			paint_background.setAntiAlias(true);
			paint_background.setStrokeWidth(4);
			paint_background.setColor(color_background);

			if (system_icons.is_loaded() && default_icon_name.size()>0) {
				auto image = system_icons.get_icon(default_icon_name);
				if(image) default_item = (*image).image;
			}
}

void ItemExplorer::compile(){

	font = SkFont(typeface, font_size, 1.0f, 0.0f);
	font.setEdging(SkFont::Edging::kAntiAlias);
	compiled = true;
}

ItemExplorer::~ItemExplorer(){

}

std::list<size_t> ItemExplorer::highlighted(){
	std::lock_guard<std::mutex> g{ get_mutex() };
	std::list<size_t> current_selected_identifiers;
	for(const auto& item : item_list)
		if(item.is_selected)
			current_selected_identifiers.push_back(item.identifier);
	return current_selected_identifiers;
}

void ItemExplorer::add(Item&& item_to_add){
	std::lock_guard<std::mutex> g{ get_mutex() };
	if(item_to_add.image.get()==nullptr)
		item_to_add.image = default_item;
	font.measureText(item_to_add.text.data(),item_to_add.text.size(),SkTextEncoding::kUTF8,&item_to_add.cached_text_size);
	to_add.push_back(std::move(item_to_add));
}

void ItemExplorer::remove(size_t identifier){
	std::lock_guard<std::mutex> g{ get_mutex() };
	to_remove.push_back(identifier);
}

drawablefunction ItemExplorer::draw(){
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
	auto lamb = [this](SkCanvas* canvas) {

		{ // do asyncronous tasks submited in previous time instants
			std::lock_guard<std::mutex> g{ get_mutex() };
			for(const auto& indent : to_remove)
				item_list.remove_if([&](const Item& item){ return item.identifier == indent;});
			to_remove = std::list<size_t>{};
			if(to_add.size()>=0){
				item_list.splice(item_list.end(),to_add);
				to_add = std::list<Item>{};
			}
		}

		SkAutoCanvasRestore restore(canvas, true);
        auto widget_rect = get_position();
		canvas->clipRect(widget_rect);

		paint_background.setColor(get_background_color());
		canvas->drawRect(widget_rect, paint_background);

		auto iterator = item_list.begin();
		auto size = item_list.size();

		SkScalar width = widget_rect.width();
		int number_of_previews_per_line = (width - 2 * ITEM_PREVIEW_EXTREMA_SPACING_LINE) / ITEM_PREVIEW_WIDTH-1;

		if (number_of_previews_per_line <= 0)
			number_of_previews_per_line = 1;
		SkScalar virtual_spacing_between_icons_line = (width - 2 * ITEM_PREVIEW_EXTREMA_SPACING_LINE - number_of_previews_per_line * ITEM_PREVIEW_WIDTH) / number_of_previews_per_line;
		virtual_spacing_between_icons_line = (virtual_spacing_between_icons_line<=0.0) ? 1 : virtual_spacing_between_icons_line;
		SkScalar spacing_between_icons_line =  ITEM_MAXIMUM_SPACING;
		uint32_t number_of_vertical_previews = size / number_of_previews_per_line
			+ (((size < 0) ^ (number_of_previews_per_line > 0)) && (size % number_of_previews_per_line));

		maximum_height = number_of_vertical_previews * (ITEM_PREVIEW_HEIGHT + ITEM_PREVIEW_EXTREMA_SPACING_COLUNM);

		SkScalar x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
		SkScalar virtual_x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
		SkScalar y = ITEM_PREVIEW_EXTREMA_SPACING_COLUNM + widget_rect.y() + current_height_offset;

		while ((iterator != item_list.end()) && (y < widget_rect.fBottom))
		{
			preview_rectangle.setXYWH(x, y, ITEM_PREVIEW_WIDTH,ITEM_PREVIEW_HEIGHT);
			iterator->current_pos = preview_rectangle;
			bool is_selected = false;
			bool is_highlighted = false;
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				is_selected = iterator->is_selected;
				is_highlighted = iterator->is_highlighted;
			}

			if (is_selected){
				paint_image_background.setColor(get_selected_color()); 
			}	else if(is_highlighted) {
				paint_image_background.setColor(get_hover_color());
			} else{
				paint_image_background.setColor(get_waiting_color());
			}

			float image_width = iterator->image->width();
			float image_height = iterator->image->height();

			float preview_width = preview_rectangle.width();
			float preview_height = preview_rectangle.height();

			float scale_factor = std::min((preview_width * 0.95f) / image_width, ITEM_IMAGE_HEIGHT / image_height);

			float init_x = (preview_width - image_width * scale_factor) / 2.0f + preview_rectangle.x();
			float init_y = preview_rectangle.y() + (ITEM_PREVIEW_HEIGHT - ITEM_IMAGE_HEIGHT);

			SkRect image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, ITEM_IMAGE_HEIGHT * 0.98f);
			canvas->drawRoundRect(preview_rectangle, 5, 5, paint_image_background);
			text_paint.setColor(get_text_color());
			canvas->drawSimpleText(iterator->text.data(),iterator->text.size(),SkTextEncoding::kUTF8,x + (ITEM_PREVIEW_WIDTH - iterator->cached_text_size.width()) / 2.0f,y + (iterator->cached_text_size.height()+ITEM_PREVIEW_HEIGHT - ITEM_IMAGE_HEIGHT)/2.0,font,text_paint);

			SkSamplingOptions options;
			canvas->drawImageRect(iterator->image, image_rectangle, options, nullptr);

			iterator++;

			x += ITEM_PREVIEW_WIDTH + spacing_between_icons_line;
			virtual_x += ITEM_PREVIEW_WIDTH + virtual_spacing_between_icons_line;
			if (virtual_x + ITEM_PREVIEW_WIDTH >= widget_rect.fRight) {
				y += ITEM_PREVIEW_HEIGHT + ITEM_PREVIEW_EXTREMA_SPACING_COLUNM;
				x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
				virtual_x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
			}
		}
	};
	return lamb;
}

callablefunction ItemExplorer::call(){
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
    auto lamb = [this](Signal sig, ConfigDraw* config) {
		bool interacted = false;
		std::visit(utilities::overloaded{
			[this,config](Empty arg) {

			},
			[this,&interacted,config](Move arg) {
				if (!get_position().contains(arg.xpos, arg.ypos)){
					std::lock_guard<std::mutex> g{get_mutex()};
					for(auto & item : item_list)
						item.is_highlighted = false;
					return;
				}
				std::lock_guard<std::mutex> g{get_mutex()};
				for(auto & item : item_list)
					if(item.current_pos.contains(arg.xpos,arg.ypos)){
						item.is_highlighted = true;
						interacted = true;
					} else {
						item.is_highlighted = false;
					}
			},
			[this,&interacted,config](Press arg) {
				if (!get_position().contains(arg.xpos, arg.ypos)){
					return;
				}
				{
					std::lock_guard<std::mutex> g{get_mutex()};
					Item* selected_item = nullptr;
					for(auto & item : item_list)
						if(item.current_pos.contains(arg.xpos,arg.ypos)){
							item.is_selected = !item.is_selected;
							selected_item = &item;
							interacted = true;
						}
					if( is_exclusive && selected_item )
						for(auto & item : item_list)
							if(&item!=selected_item)
								item.is_selected = false;
				}

				if(interacted)
					for(auto& press_callback : callbacks_press)
						press_callback(this,arg,config);
						
			},
			[this,&interacted,config](Scroll arg) {;
				if (get_position().contains(arg.xpos, arg.ypos)) {
					interacted = true;
					float increment = -arg.yoffset * 5;
					if ((current_height_offset + increment + maximum_height > get_position().height()) && (current_height_offset + increment <= 0))
						current_height_offset = current_height_offset + increment;
				}
			},
			[this,&interacted,config](Unpress arg) {

			},
			[this](Key arg) {

			},
			[this](ItemDropped arg) {;

			}},
			sig);
			return interacted;
		};
	return lamb;
}

}
}