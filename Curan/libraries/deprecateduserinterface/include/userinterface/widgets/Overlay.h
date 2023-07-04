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
		public:
			struct Info {
				post_signal_callback post_sig;
				std::shared_ptr<Container> contained;
				SkColor backgroundcolor = SK_ColorWHITE;

				Info();
			};

		private:
			std::unique_ptr<LightWeightPage> main_page;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;

			Overlay(Info drawables);
		public:
			static std::shared_ptr<Overlay> make(Info);
			std::unique_ptr<LightWeightPage> take_ownership();
		};
	}
}

#endif