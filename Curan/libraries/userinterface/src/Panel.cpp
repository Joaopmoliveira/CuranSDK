#include "userinterface/widgets/Panel.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

namespace curan{
namespace ui{


	Panel::Panel(curan::ui::IconResources& other,std::optional<curan::ui::ImageWrapper> image_wrapper) : system_icons{other} 
	{		
		background = image_wrapper;
		
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorGREEN);

		background_paint.setStyle(SkPaint::kFill_Style);
		background_paint.setAntiAlias(true);
		background_paint.setStrokeWidth(4);
		background_paint.setColor(SK_ColorBLACK);

		paint_stroke.setStyle(SkPaint::kStroke_Style);
		paint_stroke.setAntiAlias(true);
		paint_stroke.setStrokeWidth(8);
		paint_stroke.setColor(SK_ColorGREEN);

		imgfilter = SkImageFilters::Blur(10, 10, nullptr);
		bluring_paint.setImageFilter(imgfilter);
		options = SkSamplingOptions();

		const char* fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		auto typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

	}

	void Panel::insert_in_map(const curan::ui::PointCollection& future_stroke){
		strokes.try_emplace(counter,curan::ui::Stroke{future_stroke.normalized_recorded_points,inverse_homogenenous_transformation});
		++counter;
	}


	std::unique_ptr<Panel> Panel::make(curan::ui::IconResources& other,std::optional<curan::ui::ImageWrapper> image_wrapper){
		std::unique_ptr<Panel> button = std::unique_ptr<Panel>(new Panel{other,image_wrapper});
		return button;
	}

	void Panel::compile() {

	}

	void Panel::framebuffer_resize(const SkRect& new_page_size) {
		auto pos = get_position();
		background_rect = (background) ? curan::ui::compute_bounded_rectangle(pos,(*background).image->width(),(*background).image->height()) : pos;
		homogenenous_transformation = SkMatrix::MakeRectToRect(background_rect,SkRect::MakeWH(1.0,1.0),SkMatrix::ScaleToFit::kFill_ScaleToFit);
		homogenenous_transformation.invert(&inverse_homogenenous_transformation);

    	for(auto & stro : strokes)
			stro.second.container_resized(inverse_homogenenous_transformation);
		set_size(pos);
    	return ;
	}

	curan::ui::drawablefunction Panel::draw() 
	{
		auto lamb = [this](SkCanvas *canvas)
		{
			auto widget_rect = get_position();
			
			SkAutoCanvasRestore restore{canvas, true};
			canvas->drawRect(widget_rect,background_paint);

			if(background){
				auto val = *background;
				auto image_display_surface = val.image;
				background_rect = curan::ui::compute_bounded_rectangle(widget_rect,image_display_surface->width(),image_display_surface->height());
				SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
				canvas->drawImageRect(image_display_surface, background_rect, opt);
				canvas->drawRect(background_rect,paint_stroke);
			} 

			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.transformed_recorded_points.size(), current_stroke.transformed_recorded_points.data(), paint_stroke);
			
			if (is_highlighting)
			{
				double minimum = std::numeric_limits<double>::max();
				auto minimum_index = strokes.end();
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					double local = begin->second.distance(homogenenous_transformation,zoom_in.get_coordinates());
					if (minimum > local)
					{
						minimum = local;
						minimum_index = begin;
					}
					
					if(begin->second.normalized_recorded_points.size()==1) canvas->drawPoint(begin->second.begin_point,paint_stroke);
					else canvas->drawPath(begin->second.rendered_path, paint_stroke);
				}

				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					if(begin->second.normalized_recorded_points.size()==1){
						auto point = begin->second.begin_point;
						paint_square.setColor(SkColorSetARGB(60,0,0,0));
						canvas->drawCircle(SkPoint::Make(point.fX+5,point.fY+5),10,paint_square);
						paint_square.setColor(SK_ColorGREEN);
						std::string indentifier = begin->second.identifier+std::to_string(begin->first);
						canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_square);
					}else {
						auto point = begin->second.begin_point;
						paint_square.setColor(SkColorSetARGB(155,0,0,0));
						canvas->drawCircle(SkPoint::Make(point.fX+10,point.fY+10),20,paint_square);
						paint_square.setColor(SK_ColorGREEN);
						std::string indentifier = begin->second.identifier+std::to_string(begin->first);
						paint_stroke.setStrokeWidth(0.5f);
						canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_square);
						paint_stroke.setStrokeWidth(8);
					}

				}			

				if (minimum_index != strokes.end() && minimum < 0.02f)
				{
					paint_stroke.setStrokeWidth(14);
					paint_stroke.setColor(SkColorSetARGB(125,0x00, 0xFF, 0xFF));
					if(minimum_index->second.normalized_recorded_points.size()==1) canvas->drawPoint(minimum_index->second.begin_point,paint_stroke);
					else canvas->drawPath(minimum_index->second.rendered_path, paint_stroke);
					paint_stroke.setStrokeWidth(8);
					paint_stroke.setColor(SK_ColorGREEN);
				}
			}
			else
			{
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{					
					if(begin->second.normalized_recorded_points.size()==1) canvas->drawPoint(begin->second.begin_point,paint_stroke);
					else canvas->drawPath(begin->second.rendered_path, paint_stroke);
				}

				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					if(begin->second.normalized_recorded_points.size()==1){
						auto point = begin->second.begin_point;
						paint_square.setColor(SkColorSetARGB(60,0,0,0));
						canvas->drawCircle(SkPoint::Make(point.fX+5,point.fY+5),10,paint_square);
						paint_square.setColor(SK_ColorGREEN);
						std::string indentifier = begin->second.identifier+std::to_string(begin->first);
						canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_square);
					}else {
						auto point = begin->second.begin_point;
						paint_square.setColor(SkColorSetARGB(155,0,0,0));
						canvas->drawCircle(SkPoint::Make(point.fX+10,point.fY+10),20,paint_square);
						paint_square.setColor(SK_ColorGREEN);
						std::string indentifier = begin->second.identifier+std::to_string(begin->first);
						paint_stroke.setStrokeWidth(0.5f);
						canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_square);
						paint_stroke.setStrokeWidth(8);
					}

				}			
			}

			if (zoom_in)
				zoom_in.draw(canvas);
		};
		return lamb;
	}

	curan::ui::callablefunction Panel::call()
	{
		auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config){
			bool interacted = false;
			std::visit(curan::utilities::overloaded{
				[](curan::ui::Empty arg) {

				},
				[&](curan::ui::Move arg){
					
					if(!interacts(arg.xpos,arg.ypos)){
						return;
					}

					zoom_in.store_position(SkPoint::Make((float)arg.xpos, (float)arg.ypos), get_size());

					if (!is_highlighting){
						if (is_pressed){
							current_stroke.add_point(homogenenous_transformation,SkPoint::Make((float)arg.xpos, (float)arg.ypos));
						} else if (!current_stroke.empty()){
							insert_in_map(current_stroke);
							current_stroke.clear();
						}
						interacted = true;
					}
					
				},
				[&](curan::ui::Press arg){
					if(!interacts(arg.xpos,arg.ypos) || is_highlighting)
						return;
					if(current_stroke.normalized_recorded_points.size()==1){
						insert_in_map(current_stroke);
						current_stroke.clear();
					}
					current_stroke.add_point(homogenenous_transformation,SkPoint::Make((float)arg.xpos, (float)arg.ypos));
					is_pressed = true;
					interacted = true;
				},
				[](curan::ui::Scroll arg) {
				
				},
				[&](curan::ui::Unpress arg){
					is_pressed = false;
					if (!current_stroke.empty()){
						insert_in_map(current_stroke);
						current_stroke.clear();
					}
					interacted = true;
				},
				[&](curan::ui::Key arg){
					if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS){
						if (zoom_in)
							zoom_in.deactivate();
						else
							zoom_in.activate(); 
						return ;
					}

					if (arg.key == GLFW_KEY_S && arg.action == GLFW_PRESS){
						is_highlighting = !is_highlighting;
						if (!current_stroke.empty()){
							insert_in_map(current_stroke);
							current_stroke.clear();
						} 
					}
					
				},
				[](curan::ui::ItemDropped arg) {

				}},sig);
				return interacted;
		};
		return lamb;
	};
	

}
}