#ifndef CURAN_MINI_PAGE_HEADER_FILE_
#define CURAN_MINI_PAGE_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"
#include "LightWeightPage.h"

namespace curan {
	namespace ui {

		class MiniPage final : public  Drawable , public utilities::Lockable, public SignalProcessor<MiniPage> {

			std::unique_ptr<LightWeightPage> main_page;
			std::unique_ptr<LightWeightPage> replacement_main_page;
			sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(10, 10, nullptr);
			SkPaint bluring_paint;
			SkSamplingOptions options;
			SkRect previous_size;
			MiniPage(std::unique_ptr<Container> container,SkColor background);

		public:

			static std::unique_ptr<MiniPage> make(std::unique_ptr<Container> container,SkColor background);

			void compile() override;

			void construct(std::unique_ptr<Container> container,SkColor background);

			~MiniPage();

			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize(const SkRect& new_page_size) override;
		};
	}
}

#endif