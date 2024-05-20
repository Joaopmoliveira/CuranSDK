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
			explicit Overlay(std::unique_ptr<Container> contained,SkColor backgroundcolor,bool tight = false);
			explicit Overlay(std::unique_ptr<Container> contained,SkColor backgroundcolor, post_signal_callback callback,bool tight = false);
		public:

			static std::unique_ptr<Overlay> make(std::unique_ptr<Container> contained,SkColor backgroundcolor,bool tight = false);
 			static std::unique_ptr<Overlay> make(std::unique_ptr<Container> contained,post_signal_callback callback,SkColor backgroundcolor,bool tight = false);           
			std::unique_ptr<LightWeightPage> take_ownership();
		};
	}
}

#endif