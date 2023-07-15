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



add_executable(test_debug_api debug_image_orientation_and_position.cpp)

target_compile_definitions(test_debug_api PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_debug_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_debug_api
PUBLIC
renderable
imageprocessing
${ITK_LIBRARIES}
)

add_executable(convert_Plus2ITK_image convert_Plus2ITK_image.cpp)

target_compile_definitions(convert_Plus2ITK_image PRIVATE vsgXchange_FOUND)
target_compile_definitions(convert_Plus2ITK_image PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(convert_Plus2ITK_image
PUBLIC
communication
renderable
imageprocessing
${ITK_LIBRARIES}
)

add_subdirectory(volumetric)
