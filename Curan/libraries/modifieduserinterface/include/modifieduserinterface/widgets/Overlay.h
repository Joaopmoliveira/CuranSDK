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
			LightWeightPage main_page;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;
		public:
            explicit Overlay(post_signal_callback post_sig,Container&& contained,SkColor backgroundcolor);
			LightWeightPage&& take_ownership();
		};
	}
}

#endif