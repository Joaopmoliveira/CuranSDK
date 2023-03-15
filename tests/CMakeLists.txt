find_package(Vulkan REQUIRED)

add_executable(test_util test_utils.cpp)

target_link_libraries(test_util PUBLIC
utils
Pal::Sigslot
)

add_executable(test_asio test_asio.cpp)

target_link_libraries(test_asio PUBLIC
asio
utils
OpenIGTLink
)

add_executable(test_user_interface test_user_interface.cpp)

target_include_directories(test_user_interface PUBLIC $<TARGET_PROPERTY:skia,INTERFACE>)

target_link_libraries(test_user_interface PUBLIC
asio
utils
OpenIGTLink
skia
glfw 
Vulkan::Vulkan 
)

