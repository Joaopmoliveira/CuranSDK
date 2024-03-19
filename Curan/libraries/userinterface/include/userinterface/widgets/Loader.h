#ifndef CURAN_LOADER_HEADER_FILE_
#define CURAN_LOADER_HEADER_FILE_

#include <vector>
#include <atomic>
#include "LightWeightPage.h"
#include "IconResources.h"
#include <deque>
#include <memory>

namespace curan {
	namespace ui {

		class ImageDisplay;

		enum customizable{
			FLASH,
			BLUR_IN,
			BLUR_OUT
		};

		class Loader {
			std::unique_ptr<LightWeightPage> main_page;
			SkPaint bluring_paint;
			SkSamplingOptions options;
			std::string icon_identifier;
			IconResources& system_icons;
			ImageDisplay* image_display = nullptr;
			bool compiled = false;

			void compile();
			explicit Loader(std::unique_ptr<Container> contained,const std::string& icon_identifier,IconResources& system_icons,ImageDisplay* image_display);
		public:

			~Loader();

			static std::unique_ptr<Loader> make(const std::string& icon_identifier,IconResources& system_icons);
			
			std::unique_ptr<LightWeightPage> take_ownership();
		};
	}
}

#endif