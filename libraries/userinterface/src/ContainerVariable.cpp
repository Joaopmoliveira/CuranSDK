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

			};
			return lamb;
		}

		void ContainerVariable::framebuffer_resize(SkRect& pos) {

		}
	}
}