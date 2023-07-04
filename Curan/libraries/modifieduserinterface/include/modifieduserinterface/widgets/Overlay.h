#ifndef CURAN_OVERLAY_HEADER_FILE_
#define CURAN_OVERLAY_HEADER_FILE_

#include <vector>
#include <atomic>
#include "LightWeightPage.h"
#include <deque>
#include <memory>

namespace curan {
	namespace ui {

		class Overlay {
			std::unique_ptr<LightWeightPage> main_page;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;
			SkColor backgroundcolor;
			void compile();

		public:
            explicit Overlay(std::unique_ptr<Container> contained,SkColor backgroundcolor);
			std::unique_ptr<LightWeightPage> take_ownership();
		};
	}
}

#endif