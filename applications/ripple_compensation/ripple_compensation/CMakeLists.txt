add_executable(RippleReinjection main.cpp)

target_compile_definitions(RippleReinjection PRIVATE SK_VULKAN)
target_compile_definitions(RippleReinjection PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(RippleReinjection PUBLIC
utils
robotutils
renderable
)

