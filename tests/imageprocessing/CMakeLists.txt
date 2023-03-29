add_executable(test_filtering test_filtering_algorithm.cpp )

find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

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