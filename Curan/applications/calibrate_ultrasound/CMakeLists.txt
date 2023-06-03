find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(ultrasoundcalibration main.cpp 
MessageProcessing.cpp
CalibratePages.cpp
)

target_link_libraries(ultrasoundcalibration PUBLIC
userinterface
communication
imageprocessing
${ITK_LIBRARIES}
optimization
)
