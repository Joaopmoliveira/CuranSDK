#ifndef CURAN_ICONRESOURCES_HEADER_FILE_
#define CURAN_ICONRESOURCES_HEADER_FILE_

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

			bool read(const char* s);
		};

		class IconResources
		{
			bool is_initialized = false;
			std::map<std::string, Icon> icon_map;

		public:
			IconResources(std::string path_to_resources);
			bool load(std::string path_to_resources);
			void get_icon(sk_sp<SkImage>& image, std::string icon_string);
			inline bool is_loaded() {
				return is_initialized;
			}
		};
	}
}

#endif