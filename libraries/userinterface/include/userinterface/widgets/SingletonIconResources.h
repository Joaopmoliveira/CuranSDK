#ifndef CURAN_SINGLETONICONRESOURCES_HEADER_FILE_
#define CURAN_SINGLETONICONRESOURCES_HEADER_FILE_

#include <string>
#include <map>
#include "definitions/UIdefinitions.h"
#include "definitions/stb_image.h"

namespace curan {
	namespace ui {
		struct Icon
		{
			int texWidth = 0;
			int texHeight = 0;
			int texChannels = 0;
			stbi_uc* pixels = nullptr;
			SkPixmap pixmap;
			sk_sp<SkImage> image_to_display;

			static Icon read(const char* s);
		};

		struct IconResources
		{
			static IconResources* Get();
			static IconResources* Load(std::string path_to_resources);
			void GetIcon(sk_sp<SkImage>& image, std::string icon_string);
		private:
			static bool is_initialized;
			std::map<std::string, Icon> icon_map;
			IconResources(std::string path_to_resources);
		};
	}
}

#endif