add_executable(test_user_interface test_user_interface.cpp)

target_compile_definitions(test_user_interface PUBLIC SK_VULKAN)

target_link_libraries(test_user_interface PUBLIC
utils
userinterface
)

add_executable(test_library_userinterface test_library_userinterface.cpp)

target_link_libraries(test_library_userinterface PUBLIC
userinterface
)

add_executable(test_button test_button.cpp)

target_link_libraries(test_button PUBLIC
userinterface
)

add_executable(test_slider test_slider.cpp)

target_link_libraries(test_slider PUBLIC
userinterface
)

add_executable(test_textblob test_textblob.cpp)

target_link_libraries(test_textblob PUBLIC
userinterface
)

add_executable(test_openigtlinkviewer test_openigtlink_display.cpp)

target_link_libraries(test_openigtlinkviewer PUBLIC
userinterface
)

add_executable(test_container_logic test_container_logic.cpp)

target_link_libraries(test_container_logic PUBLIC
userinterface
)

add_executable(test_page test_container_with_window.cpp)

target_link_libraries(test_page PUBLIC
userinterface
)

add_executable(test_page_juntion test_openigtlink_viewer_with_button.cpp)

target_link_libraries(test_page_juntion PUBLIC
userinterface
)

add_executable(test_overlay test_overlay.cpp)

target_link_libraries(test_overlay PUBLIC
userinterface
)
