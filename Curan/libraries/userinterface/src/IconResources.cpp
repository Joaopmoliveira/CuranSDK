#include "userinterface/widgets/IconResources.h"
#include <filesystem>
#include "utils/MemoryUtils.h"
#include <iostream>

namespace curan {
namespace ui {

IconResources::IconResources(std::string path_to_resources)
{
	is_initialized = true;
	for (auto& p : std::filesystem::directory_iterator(path_to_resources))
	{
		std::string filename = p.path().string();

		int texWidth = 0;
		int texHeight = 0;
		int texChannels = 0;
		constexpr auto stb_format = STBI_rgb_alpha;
		stbi_uc* pixels = nullptr;
		pixels = stbi_load(filename.data(), &texWidth, &texHeight, &texChannels, stb_format);
		if (!pixels)
			continue;
		
		auto buffer = utilities::CopyBuffer::make_shared((char*)pixels,texWidth*texHeight*stb_format);
		curan::ui::ImageWrapper wrapper{buffer,static_cast<size_t>(texWidth),static_cast<size_t>(texHeight),SkColorType::kRGBA_8888_SkColorType};
		
		stbi_image_free(pixels);

		icon_map.emplace(std::make_pair(p.path().filename().string(), wrapper));
	}
}

std::optional<ImageWrapper> IconResources::get_icon(std::string icon_string){
	auto item_in_map = icon_map.find(icon_string);
	if (item_in_map != icon_map.end()) return item_in_map->second;
	return std::nullopt;
}

}
}