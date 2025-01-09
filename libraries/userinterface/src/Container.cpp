#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Logger.h"

namespace curan {
namespace ui {

Container::Container(const ContainerType& intype,const Arrangement& inarragement) :
	 type{intype} , 
	 arragement{inarragement}, 
	 layout_color{SK_ColorTRANSPARENT} ,
	 background_image{std::nullopt}
{
	paint_layout.setStyle(SkPaint::kFill_Style);
	paint_layout.setAntiAlias(true);
	paint_layout.setStrokeWidth(4);
	paint_layout.setColor(layout_color);
}

Container::Container(const ContainerType& intype,const Arrangement& inarragement, ImageWrapper image_wrapper) : 
		type{intype} , 
		arragement{inarragement}, 
		layout_color{SK_ColorTRANSPARENT},
		background_image{image_wrapper}
{
	paint_layout.setStyle(SkPaint::kFill_Style);
	paint_layout.setAntiAlias(true);
	paint_layout.setStrokeWidth(4);
	paint_layout.setColor(layout_color);
}

Container::Container(const ContainerType& intype,const Arrangement& inarragement, RuntimeEffect effect_wrapper) : 
		type{intype} , 
		arragement{inarragement}, 
		layout_color{SK_ColorTRANSPARENT},
		background_effect{effect_wrapper}
{
	paint_layout.setStyle(SkPaint::kFill_Style);
	paint_layout.setAntiAlias(true);
	paint_layout.setStrokeWidth(4);
	paint_layout.setColor(layout_color);
}

std::unique_ptr<Container> Container::make(const ContainerType& type,const Arrangement& arragement){
	std::unique_ptr<Container> container = std::unique_ptr<Container>(new Container{type,arragement});
	return container;
}

std::unique_ptr<Container> Container::make(const ContainerType& type, const Arrangement& arragement, ImageWrapper image_wrapper){
	std::unique_ptr<Container> container = std::unique_ptr<Container>(new Container{type,arragement,image_wrapper});
	return container;
}

std::unique_ptr<Container> Container::make(const ContainerType& type, const Arrangement& arragement, RuntimeEffect effect_wrapper){
	std::unique_ptr<Container> container = std::unique_ptr<Container>(new Container{type,arragement,effect_wrapper});
	return container;
}


Container::~Container(){
	
}

drawablefunction Container::draw(){
	if(background_image)
		return [this](SkCanvas* canvas) {
			SkRect rectangle = get_position();
			SkRect current_selected_image_rectangle = rectangle;

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kStroke_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SK_ColorGREEN);

			if(!shader_colors)	paint_layout.setColor(get_color());
			canvas->drawRect(rectangle, paint_layout);

			auto image_display_surface = (*background_image).image;
			current_selected_image_rectangle = compute_bounded_rectangle(rectangle,image_display_surface->width(),image_display_surface->height(),1.0,1.0);

			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SkColorSetARGB(0xFF,0xFF,0xFF,0xFF));
			canvas->drawImageRect(image_display_surface, current_selected_image_rectangle, opt,&paint_square);
		};
	else if(background_effect)
		return [this](SkCanvas* canvas) {
			SkRect rectangle = get_position();
			if(!shader_colors)	paint_layout.setColor(get_color());
			canvas->drawRect(rectangle, paint_layout);
			(*background_effect).draw(canvas,SkIRect::MakeXYWH(rectangle.x(),rectangle.y(),rectangle.width(),rectangle.height()));
		};
	else
		return [this](SkCanvas* canvas) {
			SkRect rectangle = get_position();
			if(!shader_colors)	paint_layout.setColor(get_color());
			canvas->drawRect(rectangle, paint_layout);
		};
}

callablefunction Container::call(){
	auto lamb = [this](Signal canvas, ConfigDraw* config) {
		return false;
	};
	return lamb;
}

void Container::framebuffer_resize(const SkRect& new_page_size){
    std::lock_guard<std::mutex> g{ get_mutex() };

	if(!compiled){
		utilities::print<utilities::Severity::major_failure>("container{:d} : was not compiled\n",(uintptr_t)this);
		throw std::runtime_error("cannot query positions while container not compiled");
	}
		

	auto iter_rect = rectangles_of_contained_layouts.begin();
	auto iter_drawables = contained_layouts.begin();
	auto my_position = get_position();
	SkScalar x_offset = my_position.fLeft;
	SkScalar y_offset = my_position.fTop;
	for (; iter_rect != rectangles_of_contained_layouts.end() && iter_drawables != contained_layouts.end(); ++iter_rect, ++iter_drawables) {
		SkRect rect = *iter_rect;
		SkRect temp = rect.MakeXYWH(my_position.width() * rect.x() + x_offset,
									my_position.height()*rect.y() + y_offset,
									my_position.width() * rect.width(),
									my_position.height() * rect.height());
		(*iter_drawables)->set_position(temp);
		(*iter_drawables)->framebuffer_resize(new_page_size);
	}

    // Define gradient start and end points
    SkPoint points[] = { SkPoint::Make(my_position.fLeft, my_position.fTop), SkPoint::Make(my_position.fRight, my_position.fTop) };

    // Create a linear gradient shader
	if(shader_colors){
		paint_layout = SkPaint{};
		paint_layout.setShader(SkGradientShader::MakeLinear(points, (*shader_colors).data(), nullptr, 2, SkTileMode::kClamp));
	}	
}

SkRect Container::minimum_size(){
    std::lock_guard<std::mutex> g{ get_mutex() };
	if(!compiled){
		utilities::print<utilities::Severity::major_failure>("container{:d} : was not compiled\n",(uintptr_t)this);
		throw std::runtime_error("cannot query positions while container not compiled");
	}
	auto iter_rect = rectangles_of_contained_layouts.begin();
	auto iter_drawables = contained_layouts.begin();
	SkRect potential_largest_container = SkRect::MakeWH(1,1);
	for (; iter_rect != rectangles_of_contained_layouts.end() && iter_drawables != contained_layouts.end(); ++iter_rect, ++iter_drawables) {
		SkRect rect = *iter_rect;
		SkRect minimum_size = (*iter_drawables)->minimum_size();
		SkRect temp = rect.MakeWH(minimum_size.width()/rect.width(),minimum_size.height()/rect.height());
		potential_largest_container.fRight = (temp.width()>potential_largest_container.width()) ? temp.width() : potential_largest_container.width();
		potential_largest_container.fBottom =  (temp.height()>potential_largest_container.height()) ? temp.height() : potential_largest_container.height();
	}
	return potential_largest_container;
}

Container& Container::linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal){
	std::lock_guard<std::mutex> g{ get_mutex() };

	if(!compiled){
		utilities::print<utilities::Severity::major_failure>("container{:d} : was not compiled\n",(uintptr_t)this);
		throw std::runtime_error("cannot query positions while container not compiled");
	}
	std::vector<drawablefunction> linearized_draw;
	std::vector<callablefunction> linearized_call;

	auto tempdraw = draw();
	auto tempcall = call();
	linearized_draw.push_back(tempdraw);
	linearized_call.push_back(tempcall);

	for (auto& drawble : contained_layouts) {
		if (drawble->is_leaf()) {
			auto tempdraw = drawble->draw();
			auto tempcall = drawble->call();
			linearized_draw.push_back(tempdraw);
			linearized_call.push_back(tempcall);
		} else {
			auto limited_raw_acess = drawble.get();
			auto interpreted = dynamic_cast<Container*>(limited_raw_acess);
			std::vector<drawablefunction> temp_linearized_draw;
			std::vector<callablefunction> temp_linearized_call;
			interpreted->linearize_container(temp_linearized_draw, temp_linearized_call);
			linearized_draw.insert(linearized_draw.end(), temp_linearized_draw.begin(), temp_linearized_draw.end());
			linearized_call.insert(linearized_call.end(), temp_linearized_call.begin(), temp_linearized_call.end());
		}
	}

	callable_draw = linearized_draw;
	callable_signal = linearized_call;
    return *(this);
}

Container& Container::operator<<(std::unique_ptr<Drawable> drawable){
	drawable->compile();
	contained_layouts.emplace_back(std::move(drawable));
    return *(this);
}

void validate_minimum_width_and_height(const std::vector<SkRect>& rectangles){
	for(const auto & rect : rectangles)
		if(rect.width()<0.000001 || rect.height()<0.000001){
			utilities::print<utilities::Severity::major_failure>("sizes are incorrect ({:f},{:f})\n",rect.width(),rect.height());
			throw std::runtime_error("the size of one or more of the supplied widgets is too close to zero");
		}
}

void Container::compile(){
	if (contained_layouts.size() == 0)
		return;
	SkScalar packet_width = 1.0f / contained_layouts.size();
	if(type == ContainerType::VARIABLE_CONTAINER){
		if(contained_layouts.size()!=rectangles_of_contained_layouts.size())
			throw std::runtime_error("missmatch between the number of contained layout and rectangles");
		compiled = true;
		validate_minimum_width_and_height(rectangles_of_contained_layouts);
		return;
	}
	switch (arragement) {
	case Arrangement::HORIZONTAL:
		if ((divisions.size() != 0) && (divisions.size() - 1 == contained_layouts.size())) {
			for (int i = 0; i < divisions.size() - 1; ++i) {
				SkRect widget_rect = SkRect::MakeLTRB(divisions[i], 0, divisions[i + 1], 1);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		} else {
			for (SkScalar left_position = 0.0; left_position < 1.0; left_position += packet_width) {
				SkRect widget_rect = SkRect::MakeLTRB(left_position, 0, left_position + packet_width,1);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		}
		break;
	case Arrangement::VERTICAL:
		if ((divisions.size() != 0) && (divisions.size() - 1 == contained_layouts.size())) {
			for (int i = 0; i < divisions.size() - 1; ++i) {
				SkRect widget_rect = SkRect::MakeLTRB(0, divisions[i], 1, divisions[i + 1]);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		} else {
			for (SkScalar top_position = 0.0; top_position < 1.0; top_position += packet_width) {
				SkRect widget_rect = SkRect::MakeLTRB(0, top_position, 1, top_position + packet_width);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		}
		break;
	default :
		utilities::print<utilities::Severity::major_failure>("container{:d} : the Arrangement must be either vertical or horizontal\n",(uintptr_t)this);
		throw std::runtime_error("the Arrangement must be either vertical or horizontal");
		break;
	}
	compiled = true;
	validate_minimum_width_and_height(rectangles_of_contained_layouts);
}

}
}