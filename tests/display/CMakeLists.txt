find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

# ---------------------------------------

add_executable(modified_test_user_interface test_user_interface.cpp)

target_compile_definitions(modified_test_user_interface PRIVATE SK_VULKAN)
target_compile_definitions(modified_test_user_interface PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_user_interface PUBLIC
utils
userinterface
)

# ---------------------------------------

add_executable(modified_test_library_userinterface test_library_userinterface.cpp)

target_compile_definitions(modified_test_library_userinterface PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_library_userinterface PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_button test_button.cpp)

target_compile_definitions(modified_test_button PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_button PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_slider test_slider.cpp)

target_compile_definitions(modified_test_slider PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_slider PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_textblob test_textblob.cpp)

target_compile_definitions(modified_test_textblob PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_textblob PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_openigtlinkviewer test_openigtlink_display.cpp)

target_compile_definitions(modified_test_openigtlinkviewer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_openigtlinkviewer PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_container_logic test_container_logic.cpp)

target_compile_definitions(modified_test_container_logic PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_container_logic PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_page test_container_with_window.cpp)

target_compile_definitions(modified_test_page PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_page PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_page_juntion test_openigtlink_viewer_with_button.cpp)

target_compile_definitions(modified_test_page_juntion PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_page_juntion PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_overlay_api test_overlay_api.cpp)

target_compile_definitions(modified_test_overlay_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_overlay_api PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_test_multiple_sliders test_multiple_sliders.cpp)

target_compile_definitions(modified_test_multiple_sliders PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_test_multiple_sliders PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_skia_playground skia_playground.cpp)

target_compile_definitions(modified_skia_playground PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_skia_playground PUBLIC
userinterface
)

# ---------------------------------------

add_executable(modified_skia_playground2 skia_playground2.cpp)

target_compile_definitions(modified_skia_playground2 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(modified_skia_playground2 PUBLIC
userinterface
${ITK_LIBRARIES}
)

# ---------------------------------------

add_executable(test_image_visualization test_image_visualization.cpp)

target_compile_definitions(test_image_visualization PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_image_visualization
PUBLIC
userinterface
${ITK_LIBRARIES}
)

# ---------------------------------------

add_executable(test_minimum_window_size_computation test_minimum_window_size_computation.cpp)

target_compile_definitions(test_minimum_window_size_computation PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_minimum_window_size_computation PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_minimum_window_with_glfw test_gflw_plus_minimum_window_size.cpp)

target_compile_definitions(test_minimum_window_with_glfw PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_minimum_window_with_glfw PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_resampling_gui test_multiple_plane_specification.cpp)

target_compile_definitions(test_resampling_gui PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_resampling_gui
PUBLIC
userinterface
Eigen3::Eigen
${ITK_LIBRARIES}
)

# ---------------------------------------

add_executable(test_panel_api capture_trajectories_from_screen.cpp)

target_compile_definitions(test_panel_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_panel_api
PUBLIC
${ITK_LIBRARIES}
userinterface
)

# ---------------------------------------

add_executable(test_collect_demonstration_data collect_demonstration_data.cpp)

target_compile_definitions(test_collect_demonstration_data PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_collect_demonstration_data PUBLIC
userinterface
)

# ---------------------------------------

add_executable(study_skia_transformations study_skia_transformations.cpp)

target_compile_definitions(study_skia_transformations PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(study_skia_transformations PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_slidingpanel_api panel_with_slider.cpp)

target_compile_definitions(test_slidingpanel_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_slidingpanel_api
PUBLIC
${ITK_LIBRARIES}
userinterface
)

# ---------------------------------------

add_executable(test_minipage_with_panelslider test_minipage_with_panelslider.cpp)

target_compile_definitions(test_minipage_with_panelslider PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_minipage_with_panelslider
PUBLIC
${ITK_LIBRARIES}
userinterface
)

# ---------------------------------------

add_executable(test_writable_text_panel test_writable_text_panel.cpp)

target_compile_definitions(test_writable_text_panel PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_writable_text_panel
PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_imutable_text_panel test_imutable_text_panel.cpp)

target_compile_definitions(test_imutable_text_panel PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_imutable_text_panel
PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_itk_resampling test_itk_resampling.cpp)

target_compile_definitions(test_itk_resampling PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_itk_resampling
PUBLIC
userinterface
Eigen3::Eigen
${ITK_LIBRARIES}
)

# ---------------------------------------

add_executable(test_item_previewer item_previewer_testing.cpp)

target_compile_definitions(test_item_previewer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_item_previewer
PUBLIC
userinterface
)

# ---------------------------------------

add_executable(test_interpolation_realtime test_interpolation_realtime.cpp)

target_compile_definitions(test_interpolation_realtime PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_interpolation_realtime
PUBLIC
userinterface
Eigen3::Eigen
${ITK_LIBRARIES}
)

# ----------------------------------------

add_executable(test_overlay_loader test_loader_overlay.cpp)

target_compile_definitions(test_overlay_loader PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_overlay_loader
PUBLIC
userinterface
)


# ----------------------------------------

add_executable(test_plot_graph_realtime test_plot_graph_realtime.cpp)

target_compile_definitions(test_plot_graph_realtime PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_plot_graph_realtime
PUBLIC
userinterface
)

# ----------------------------------------

add_executable(test_sequencial_activation test_sequencial_activation.cpp)

target_compile_definitions(test_sequencial_activation PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_sequencial_activation
PUBLIC
userinterface
)

# ----------------------------------------

add_executable(test_specification_of_geometris_with_cube_intersection test_specification_of_geometris_with_cube_intersection.cpp)

target_compile_definitions(test_specification_of_geometris_with_cube_intersection PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_specification_of_geometris_with_cube_intersection
PUBLIC
${ITK_LIBRARIES}
userinterface
)


# ----------------------------------------

add_executable(test_memory_signal test_memory_signal.cpp)

target_compile_definitions(test_memory_signal PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_memory_signal
PUBLIC
${ITK_LIBRARIES}
userinterface
)

# ----------------------------------------

add_executable(test_multple_sliderplanes test_multple_sliderplanes.cpp)

target_compile_definitions(test_multple_sliderplanes PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_multple_sliderplanes
PUBLIC
${ITK_LIBRARIES}
userinterface
)

# ----------------------------------------

add_executable(test_implementation_of_task_tracker test_implementation_of_task_tracker.cpp)

target_compile_definitions(test_implementation_of_task_tracker PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_implementation_of_task_tracker
PUBLIC
${ITK_LIBRARIES}
userinterface
)


# ----------------------------------------

add_executable(test_dicom_compliant_display test_dicom_compliant_display.cpp)

target_compile_definitions(test_dicom_compliant_display PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_dicom_compliant_display
PUBLIC
${ITK_LIBRARIES}
userinterface
geometry
)
