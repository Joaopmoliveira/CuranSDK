find_package(ITK COMPONENTS ITKCommon ITKIOImageBase ITKIOGDCM ITKIOJPEG ITKIOTIFF ITKImageIO REQUIRED)

include(${ITK_USE_FILE})

add_executable(ultrasoundcalibration main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_compile_definitions(ultrasoundcalibration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(ultrasoundcalibration PUBLIC
userinterface
communication
imageprocessing
${ITK_LIBRARIES}
optimization
)
