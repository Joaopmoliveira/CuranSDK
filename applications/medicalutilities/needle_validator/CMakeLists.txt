find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(NeedleCalibration 
main.cpp 
MessageProcessing.cpp
)

target_compile_definitions(NeedleCalibration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(NeedleCalibration PUBLIC
${ITK_LIBRARIES}
imageprocessing
userinterface
communication
utils
)
