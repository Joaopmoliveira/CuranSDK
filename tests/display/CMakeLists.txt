add_executable(test_user_interface test_user_interface.cpp)

target_compile_definitions(test_user_interface PRIVATE SK_VULKAN)
target_compile_definitions(test_user_interface PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_user_interface PUBLIC
utils
userinterface
)

add_executable(test_library_userinterface test_library_userinterface.cpp)

target_compile_definitions(test_library_userinterface PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_library_userinterface PUBLIC
userinterface
)

add_executable(test_button test_button.cpp)

target_compile_definitions(test_button PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_button PUBLIC
userinterface
)

add_executable(test_slider test_slider.cpp)

target_compile_definitions(test_slider PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_slider PUBLIC
userinterface
)

add_executable(test_textblob test_textblob.cpp)

target_compile_definitions(test_textblob PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_textblob PUBLIC
userinterface
)

add_executable(test_openigtlinkviewer test_openigtlink_display.cpp)

target_compile_definitions(test_openigtlinkviewer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_openigtlinkviewer PUBLIC
userinterface
)

add_executable(test_container_logic test_container_logic.cpp)

target_compile_definitions(test_container_logic PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_container_logic PUBLIC
userinterface
)

add_executable(test_page test_container_with_window.cpp)

target_compile_definitions(test_page PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_page PUBLIC
userinterface
)

add_executable(test_page_juntion test_openigtlink_viewer_with_button.cpp)

target_compile_definitions(test_page_juntion PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_page_juntion PUBLIC
userinterface
)

add_executable(test_overlay_api test_overlay_api.cpp)

target_compile_definitions(test_overlay_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_overlay_api PUBLIC
userinterface
)

add_executable(test_multiple_sliders test_multiple_sliders.cpp)

target_compile_definitions(test_multiple_sliders PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_multiple_sliders PUBLIC
userinterface
)

add_executable(skia_playground skia_playground.cpp)

target_compile_definitions(skia_playground PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(skia_playground PUBLIC
userinterface
)