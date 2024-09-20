find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(VolumeCropper
main.cpp 
BoundingBox.cpp
UserInterface.cpp
)

target_compile_definitions(VolumeCropper PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(VolumeCropper
PUBLIC
${ITK_LIBRARIES}
userinterface
Eigen3::Eigen
)
