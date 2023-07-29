#ifndef CURAN_PAGE_HEADER_FILE_
#define CURAN_PAGE_HEADER_FILE_

#include <vector>
#include <atomic>
#include "LightWeightPage.h"
#include <deque>

namespace curan {
	namespace ui {

		class Overlay;

		class Page {
			std::unique_ptr<LightWeightPage> main_page;
			std::deque<std::unique_ptr<LightWeightPage>> page_stack;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;

        public:

			explicit Page(std::unique_ptr<Container> container,SkColor background);

			Page& draw(SkCanvas* canvas);

			bool propagate_signal(Signal sig, ConfigDraw* config);

			Page& propagate_size_change(SkRect& new_size);

			Page& pop();

			Page& stack(std::unique_ptr<Overlay> overlay);

			inline SkRect minimum_size(){
				return main_page->minimum_size();
			}

			inline Page& set_dirtyness(bool var) {
				if (page_stack.empty())
					main_page->set_dirtyness(var);
				else
					page_stack.front()->set_dirtyness(var);
                return *(this);
			}


		};
	}
}

#endif