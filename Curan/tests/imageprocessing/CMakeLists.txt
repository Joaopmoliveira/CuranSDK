find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(test_filtering test_filtering_algorithm.cpp )

target_link_libraries(test_filtering PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_itk_canny test_itk_canny.cpp)

target_link_libraries(test_itk_canny PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_itk_circle_extraction test_hough_circle.cpp)

target_link_libraries(test_itk_circle_extraction PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_wrapped_circle test_wrapped_circle.cpp)

target_link_libraries(test_wrapped_circle PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_manel_itk test_itk_manel.cpp)

target_compile_definitions(test_manel_itk PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_manel_itk PUBLIC
imageprocessing
${ITK_LIBRARIES}
)


add_executable(test_volume_reconstruction test_volume_creation.cpp)

target_compile_definitions(test_volume_reconstruction PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_volume_reconstruction PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_volume_reconstruction
PUBLIC
renderable
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_new_volume_reconstruction test_new_volume_reconstruction_api.cpp)

target_compile_definitions(test_new_volume_reconstruction PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_new_volume_reconstruction PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_new_volume_reconstruction
PUBLIC
renderable
imageprocessing
${ITK_LIBRARIES}
)