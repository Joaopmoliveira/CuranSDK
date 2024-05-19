find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(TemporalCalibration main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_compile_definitions(TemporalCalibration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(TemporalCalibration PUBLIC
${ITK_LIBRARIES}
imageprocessing
userinterface
communication
optimization
)
