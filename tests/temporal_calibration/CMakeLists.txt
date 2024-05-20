find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(temporal_calibration_tests alvaro_tests.cpp)

target_compile_definitions(temporal_calibration_tests PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(temporal_calibration_tests PUBLIC

${ITK_LIBRARIES}
)
