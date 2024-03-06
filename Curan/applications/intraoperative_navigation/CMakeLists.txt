find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(InteroperativeNavigation 
main.cpp 
MessageProcessing.cpp
LevelImplementation.cpp
DicomLoading.cpp
InteroperativePages.cpp
)

target_compile_definitions(InteroperativeNavigation PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(InteroperativeNavigation PUBLIC
${ITK_LIBRARIES}
imageprocessing
userinterface
communication
)
