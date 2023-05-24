#include "userinterface/widgets/IconResources.h"

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
		Icon icon;
		icon.read(s.c_str());
		icon_map.emplace(std::make_pair(filename, icon));
	}
}

bool IconResources::load(std::string path_to_resources)
{
	static IconResources icon_loader{ path_to_resources };
	//TODO: this needs further attention, I have no clue what I was thinking
	return &icon_loader;
}

void IconResources::get_icon(sk_sp<SkImage>& image, std::string icon_string)
{
	auto item_in_map = icon_map.find(icon_string);
	if (item_in_map != icon_map.end()) {
		image = item_in_map->second.image_to_display;
	}
}

bool Icon::read(const char* s)
{
	pixels = stbi_load(s, &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
	if (!pixels) {
		return false;
	}
	SkImageInfo information = SkImageInfo::Make(texWidth, texHeight, kBGRA_8888_SkColorType, kUnpremul_SkAlphaType);
	pixmap = SkPixmap(information, pixels, texWidth * NUMBER_BYTES_PER_PIXEL);
	image_to_display = SkImage::MakeFromRaster(pixmap, nullptr, nullptr);
	return true;
}

}
}