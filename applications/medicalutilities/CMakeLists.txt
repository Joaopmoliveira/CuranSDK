add_subdirectory(extract_intersections_ultrasound)
add_subdirectory(usdifferencemean)
add_subdirectory(image_cropper)
add_subdirectory(needle_validator)

if(WAS_PLUS_SPECIFIED)
	add_subdirectory(data_replayer_for_videos)
endif()


