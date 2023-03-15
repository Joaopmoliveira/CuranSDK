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

target_link_libraries(test_user_interface PUBLIC
utils
userinterface
)

add_executable(test_library_userinterface test_library_userinterface.cpp)

target_link_libraries(test_library_userinterface PUBLIC
utils
userinterface
)
