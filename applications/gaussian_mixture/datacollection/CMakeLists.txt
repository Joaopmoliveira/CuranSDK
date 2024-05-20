add_executable(DataCollection main.cpp)

target_compile_definitions(DataCollection PRIVATE SK_VULKAN)
target_compile_definitions(DataCollection PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(DataCollection PUBLIC
utils
robotutils
renderable
)
