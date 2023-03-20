#include "userinterface/widgets/ContainerVariable.h"

namespace curan {
	namespace ui {
		ContainerVariable::ContainerVariable(Info& info) {
			paint_layout =info.paint_layout;
			arrangment = info.arrangement;
			contained_layouts = info.layouts;
			rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
		}

		std::shared_ptr<ContainerVariable> ContainerVariable::make(Info& info) {
			return std::make_shared<ContainerVariable>(info);
		}

		drawablefunction ContainerVariable::draw() {
			auto lamb = [this](SkCanvas* canvas) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				SkRect rectangle = get_position();
				canvas->drawRect(rectangle, paint_layout);
			};
			return lamb;
		}

		callablefunction ContainerVariable::call() {
			auto lamb = [this](Signal canvas) {
				return false;
			};
			return lamb;
		}

		void ContainerVariable::framebuffer_resize() {
			std::lock_guard<std::mutex> g{ get_mutex() };
			auto iter_rect = rectangles_of_contained_layouts.begin();
			auto iter_drawables = contained_layouts.begin();
			auto my_position = get_position();
			double x_accumulator = 0.0;
			double y_accumulator = 0.0;
			for (; iter_rect != rectangles_of_contained_layouts.end() && iter_drawables != contained_layouts.end(); ++iter_rect, ++iter_drawables) {
				SkRect rect = *iter_rect;
				SkRect temp = rect.MakeXYWH(my_position.x() + x_accumulator,
					my_position.y() + y_accumulator,
					my_position.width() * rect.width(),
					my_position.height() * rect.height());
				x_accumulator += rect.x() * my_position.width();
				y_accumulator += rect.y() * my_position.height();
				(*iter_drawables)->set_position(temp);
				(*iter_drawables)->framebuffer_resize();
			}
		}
	}
}