find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(test_phantom_line_segmentation main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_compile_definitions(test_phantom_line_segmentation PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(test_phantom_line_segmentation PUBLIC
${ITK_LIBRARIES}
imageprocessing
userinterface
communication
optimization
)
