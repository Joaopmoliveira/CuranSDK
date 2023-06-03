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
		public:
			struct Info {
				std::shared_ptr<Container> contained;
				SkColor backgroundcolor = SK_ColorWHITE;
			};

		private:	
			std::unique_ptr<LightWeightPage> main_page;
			std::deque<std::unique_ptr<LightWeightPage>> page_stack;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;

			Page(Info drawables);

		public:
			static std::shared_ptr<Page> make(Info);

			void draw(SkCanvas* canvas);

			bool propagate_signal(Signal sig, ConfigDraw* config);

			void propagate_size_change(SkRect& new_size);

			void pop();

			void stack(std::shared_ptr<Overlay> overlay);

			inline void set_dirtyness(bool var) {
				if (page_stack.empty())
					main_page->set_dirtyness(var);
				else
					page_stack.front()->set_dirtyness(var);
			}


		};
	}
}

#endif