#ifndef LOAD_VOLUME_HEADER
#define LOAD_VOLUME_HEADER

#include "common_includes.h"

std::optional<ImageType::Pointer> get_volume(std::string path);

std::vector<std::string> get_representative_uids(std::string path);

std::optional<ImageType::Pointer> get_representative_series_image(std::string path,std::string seriesIdentifier);

std::optional<ImageType::Pointer> load_volume_from_selected_uid(std::string path,std::string seriesIdentifier);

#endif