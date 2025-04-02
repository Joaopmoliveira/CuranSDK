set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
if(${PORT} MATCHES "expat|libjpeg-turbo|libpng|libwebp|opengl|zlib|vulkan-headers|vulkan-memory-allocator|glslang|vulkan|xcb|imgui|vulkan-binding|implot|vsg|vsgimgui|skia|pugixml|assimp")
	set(VCPKG_CRT_LINKAGE dynamic)	
	set(VCPKG_LIBRARY_LINKAGE static )
else()
	set(VCPKG_CRT_LINKAGE dynamic)	
	set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()














