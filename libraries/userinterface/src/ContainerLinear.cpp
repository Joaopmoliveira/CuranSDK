#include "userinterface/widgets/ContainerLinear.h"

namespace curan {
	namespace ui {
		ContainerLinear::ContainerLinear(Info& info) {
			paint_layout = info.paint_layout;
			arrangment = info.arrangement;
			contained_layouts = info.layouts;

			if (contained_layouts.size() == 0)
				return;
			SkScalar packet_width = 1.0f / contained_layouts.size();

			switch (arrangment) {
			case Arrangement::HORIZONTAL:
				if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_layouts.size())) {
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(info.divisions[i], 0, info.divisions[i + 1], 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
				}
				else {
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(0, info.divisions[i], 1, info.divisions[i + 1]);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
				}
				break;
			case Arrangement::VERTICAL:
				if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_layouts.size())) {
					for (SkScalar left_position = 0.0; left_position < 1.0; left_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(left_position, 0, left_position + packet_width, 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
				}
				else {
					for (SkScalar top_position = 0.0; top_position < 1.0; top_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(0, top_position, 1, top_position + packet_width);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
				}
				break;
			default:

				break;
			}
		}

		std::shared_ptr<ContainerLinear> ContainerLinear::make(Info& info) {
			return std::make_shared<ContainerLinear>(info);
		}

		drawablefunction ContainerLinear::draw() {
			auto lamb = [this](SkCanvas* canvas) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				SkRect rectangle = get_position();
				canvas->drawRect(rectangle, paint_layout);
			};
			return lamb;
		}

		callablefunction ContainerLinear::call() {
			auto lamb = [this](Signal canvas) {
				
			};
			return lamb;
		}

		void ContainerLinear::framebuffer_resize(SkRect& pos) {
		
		}
	}
}