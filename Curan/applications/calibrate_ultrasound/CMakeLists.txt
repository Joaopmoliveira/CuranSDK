find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(ultrasoundcalibration main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_compile_definitions(ultrasoundcalibration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

message(${ITK_LIBRARIES})

target_link_libraries(ultrasoundcalibration PUBLIC
userinterface
communication
imageprocessing
${ITK_LIBRARIES}
optimization
)
