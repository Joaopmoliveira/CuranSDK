find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(UltrasoundMeanDifference main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_compile_definitions(UltrasoundMeanDifference PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(UltrasoundMeanDifference PUBLIC
${ITK_LIBRARIES}
imageprocessing
userinterface
communication
optimization
)
