#include "userinterface/widgets/ItemExplorer.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

ItemExplorer::ItemExplorer(){

			color_background = SK_ColorBLACK;
			color_hover = SK_ColorDKGRAY;
			color_selected = SkColorSetARGB(255, 77, 195, 255);
			color_waiting = SkColorSetARGB(255, 217, 217, 217);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			paint_image_background.setStyle(SkPaint::kFill_Style);
			paint_image_background.setAntiAlias(true);
			paint_image_background.setStrokeWidth(4);
			paint_image_background.setColor(color_waiting);

			paint_background.setStyle(SkPaint::kFill_Style);
			paint_background.setAntiAlias(true);
			paint_background.setStrokeWidth(4);
			paint_background.setColor(color_background);
}

void ItemExplorer::compile(){

	font = SkFont(typeface, font_size, 1.0f, 0.0f);
	font.setEdging(SkFont::Edging::kAntiAlias);
	compiled = true;
}

ItemExplorer::~ItemExplorer(){

}

std::list<size_t> ItemExplorer::highlighted(){
	return current_selected_identifiers;
}

void ItemExplorer::add(Item item_to_add,size_t identifier){

}

void ItemExplorer::remove(size_t identifier){

}

drawablefunction ItemExplorer::draw(){
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
	auto lamb = [this](SkCanvas* canvas) {
		SkAutoCanvasRestore restore(canvas, true);
        	auto widget_rect = get_position();
			canvas->clipRect(widget_rect);

			paint_background.setColor(get_background_color());
			canvas->drawRect(widget_rect, paint_background);

			auto iterator = item_list.begin();
			auto size = item_list.size();

			SkScalar width = widget_rect.width();
			uint32_t number_of_previews_per_line = (width - 2 * ITEM_PREVIEW_EXTREMA_SPACING_LINE) / ITEM_PREVIEW_WIDTH - 1;
			if (number_of_previews_per_line == 0)
				number_of_previews_per_line = 1;
			SkScalar spacing_between_icons_line = (width - 2 * ITEM_PREVIEW_EXTREMA_SPACING_LINE - (number_of_previews_per_line + 1) * ITEM_PREVIEW_WIDTH) / number_of_previews_per_line;
			uint32_t number_of_vertical_previews = size / number_of_previews_per_line
				+ (((size < 0) ^ (number_of_previews_per_line > 0)) && (size % number_of_previews_per_line));

			maximum_height = number_of_vertical_previews * (ITEM_PREVIEW_HEIGHT + ITEM_PREVIEW_EXTREMA_SPACING_COLUNM);

			SkScalar x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
			SkScalar y = ITEM_PREVIEW_EXTREMA_SPACING_COLUNM + widget_rect.y() + current_height_offset;

			size_t index = 0;
			while ((iterator != item_list.end()) && (y < widget_rect.fBottom))
			{
				Item& item_to_draw = *iterator;
				int index_of_item = index;
				preview_rectangle.setXYWH(x, y, ITEM_PREVIEW_WIDTH,ITEM_PREVIEW_HEIGHT);

				auto item_found = std::find(current_selected_identifiers.begin(), current_selected_identifiers.end(), index_of_item);

				if (item_found != current_selected_identifiers.end()){
					paint_image_background.setColor(color_hover);
				}	else {
					paint_image_background.setColor(color_waiting);
				}

				float image_width = item_to_draw.image->width();
				float image_height = item_to_draw.image->height();

				float preview_width = preview_rectangle.width();
				float preview_height = preview_rectangle.height();

				float scale_factor = std::min((preview_width * 0.95f) / image_width, ITEM_IMAGE_HEIGHT / image_height);

				float init_x = (preview_width - image_width * scale_factor) / 2.0f + preview_rectangle.x();
				float init_y = preview_rectangle.y() + (ITEM_PREVIEW_HEIGHT - ITEM_IMAGE_HEIGHT);

				SkRect image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, ITEM_IMAGE_HEIGHT * 0.98f);
				canvas->drawRoundRect(preview_rectangle, 5, 5, paint_image_background);

				SkRect bound_individual_name = item_to_draw.text->bounds();

				canvas->drawTextBlob(item_to_draw.text.get(), x + (ITEM_PREVIEW_WIDTH - bound_individual_name.width()) / 2.0f, y + bound_individual_name.height(),text_paint);

				SkSamplingOptions options;
				canvas->drawImageRect(item_to_draw.image, image_rectangle, options, nullptr);

				iterator++;

				x += ITEM_PREVIEW_WIDTH + spacing_between_icons_line;

				if (x + ITEM_PREVIEW_WIDTH >= widget_rect.fRight) {
					y += ITEM_PREVIEW_HEIGHT + ITEM_PREVIEW_EXTREMA_SPACING_COLUNM;
					x = ITEM_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
				}
				++index;
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
                interacted = true;
				current_mouse_position = arg;
			},
			[this,&interacted,config](Press arg) {
				if (get_position().contains(arg.xpos, arg.ypos)){
					interacted = true;
				}
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