#include "userinterface/widgets/SingletonIconResources.h"

#include <filesystem>

namespace curan {
	namespace ui {
		IconResources::IconResources(std::string path_to_resources)
		{
			is_initialized = true;
			for (auto& p : std::filesystem::directory_iterator(path_to_resources))
			{
				std::string s = p.path().string();
				std::string filename = p.path().filename().string();
				auto icon = Icon::read(s.c_str());
				icon_map.emplace(std::make_pair(filename, icon));
			}
		}

		bool IconResources::is_initialized = false;

		IconResources* IconResources::Get()
		{
			if (is_initialized)
				return Load("");
			else
				return nullptr;
		}

		IconResources* IconResources::Load(std::string path_to_resources)
		{
			static IconResources icon_loader{ path_to_resources };
			return &icon_loader;
		}

		void IconResources::GetIcon(sk_sp<SkImage>& image, std::string icon_string)
		{
			auto item_in_map = icon_map.find(icon_string);
			if (item_in_map != icon_map.end()) {
				image = item_in_map->second.image_to_display;
			}
		}

		Icon Icon::read(const char* s)
		{
			Icon icon;
			icon.pixels = stbi_load(s, &icon.texWidth, &icon.texHeight, &icon.texChannels, STBI_rgb_alpha);
			if (!icon.pixels) {
				throw std::runtime_error("failed to load texture image!");
			}
			SkImageInfo information = SkImageInfo::Make(icon.texWidth, icon.texHeight, kBGRA_8888_SkColorType, kUnpremul_SkAlphaType);
			icon.pixmap = SkPixmap(information, icon.pixels, icon.texWidth * 32);
			icon.image_to_display = SkImage::MakeFromRaster(icon.pixmap, nullptr, nullptr);
			return icon;
		}

	}
}