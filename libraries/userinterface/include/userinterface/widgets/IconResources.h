#ifndef CURAN_ICONRESOURCES_HEADER_FILE_
#define CURAN_ICONRESOURCES_HEADER_FILE_

#include <string>
#include <map>
#include <optional>
#include "definitions/UIdefinitions.h"
#include "ImageWrapper.h"

namespace curan {
namespace ui {

/*

*/

class IconResources{
public:
	IconResources(std::string path_to_resources);
	std::optional<ImageWrapper> get_icon(std::string icon_string);
	inline bool is_loaded() {
		return is_initialized;
	}

private:
	bool is_initialized = false;
	std::map<std::string, ImageWrapper> icon_map;
};

}
}

#endif