#ifndef CURAN_PAGE_HEADER_FILE_
#define CURAN_PAGE_HEADER_FILE_

#include <vector>
#include <atomic>
#include "LightWeightPage.h"
#include <deque>
#include "utils/Overloading.h"

namespace curan {
	namespace ui {

		class Overlay;
		class Loader;
		class Window;

		class Page {
			std::unique_ptr<LightWeightPage> main_page;
			using pointer_type = std::variant<std::unique_ptr<LightWeightPage>,std::shared_ptr<LightWeightPage>>;
			std::deque<pointer_type> page_stack;
			
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;
			SkRect previous_size;
			std::mutex mut;
        public:

			explicit Page(std::unique_ptr<Container> container,SkColor background);

			Page& draw(SkCanvas* canvas);

			bool propagate_signal(Signal sig, ConfigDraw* config);

			void propagate_heartbeat(ConfigDraw* config);

			Page& propagate_size_change(const SkRect& new_size);

			Page& pop();

			Page& stack(std::unique_ptr<Loader> loader);
			Page& stack(std::unique_ptr<Overlay> overlay);
			Page& replace_all(std::unique_ptr<Overlay> overlay);
			Page& replace_last(std::unique_ptr<Overlay> overlay);

			/* avoid using these since it makes your own logic and lifetime management harder to follow */
			Page& stack(std::shared_ptr<LightWeightPage> overlay);
			Page& replace_all(std::shared_ptr<LightWeightPage> overlay);
			Page& replace_last(std::shared_ptr<LightWeightPage> overlay);

			Page& clear_overlays();

			void update_page(const Window* window);

			inline SkRect minimum_size(){
				return main_page->minimum_size();
			}

			inline Page& set_dirtyness(bool var) {
				if (page_stack.empty())
					main_page->set_dirtyness(var);
				else
					std::visit([var](auto&& arg){ arg->set_dirtyness(var); }, page_stack.back());
                return *(this);
			}


		};
	}
}

#endif