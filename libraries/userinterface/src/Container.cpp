#include "userinterface/widgets/Container.h"

namespace curan {
namespace ui {

Container::Container(InfoLinearContainer& info) {
	paint_layout = info.paint_layout;
	contained_layouts = info.layouts;

	if (contained_layouts.size() == 0)
		return;
	SkScalar packet_width = 1.0f / contained_layouts.size();

	switch (info.arrangement) {
	case Arrangement::HORIZONTAL:
		if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_layouts.size())) {
			for (int i = 0; i < info.divisions.size() - 1; ++i) {
				SkRect widget_rect = SkRect::MakeLTRB(info.divisions[i], 0, info.divisions[i + 1], 1);
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
		if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_layouts.size())) {
			for (int i = 0; i < info.divisions.size() - 1; ++i) {
				SkRect widget_rect = SkRect::MakeLTRB(0, info.divisions[i], 1, info.divisions[i + 1]);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		} else {
			for (SkScalar top_position = 0.0; top_position < 1.0; top_position += packet_width) {
				SkRect widget_rect = SkRect::MakeLTRB(0, top_position, 1, top_position + packet_width);
				rectangles_of_contained_layouts.push_back(widget_rect);
			}
		}
		break;
	default:
		throw std::runtime_error("received unknown arrangement");
		break;
	}
}

Container::Container(InfoVariableContainer& info) {
	paint_layout = info.paint_layout;
	contained_layouts = info.layouts;
	rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
}

std::shared_ptr<Container> Container::make(InfoLinearContainer& info) {
	return std::make_shared<Container>(info);
}

std::shared_ptr<Container> Container::make(InfoVariableContainer& info) {
	return std::make_shared<Container>(info);
}

drawablefunction Container::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		SkRect rectangle = get_position();
		canvas->drawRect(rectangle, paint_layout);
	};
	return lamb;
}

callablefunction Container::call() {
	auto lamb = [this](Signal canvas) {
		return false;
	};
	return lamb;
}

void Container::framebuffer_resize() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	auto iter_rect = rectangles_of_contained_layouts.begin();
	auto iter_drawables = contained_layouts.begin();
	auto my_position = get_position();
	double x_offset = my_position.fLeft;
	double y_offset = my_position.fTop;
	for (; iter_rect != rectangles_of_contained_layouts.end() && iter_drawables != contained_layouts.end(); ++iter_rect, ++iter_drawables) {
		SkRect rect = *iter_rect;
		SkRect temp = rect.MakeXYWH(my_position.width() * rect.x() + x_offset,
									my_position.height()*rect.y() + y_offset,
									my_position.width() * rect.width(),
									my_position.height() * rect.height());

		(*iter_drawables)->set_position(temp);
		(*iter_drawables)->framebuffer_resize();
	}
}

}
}