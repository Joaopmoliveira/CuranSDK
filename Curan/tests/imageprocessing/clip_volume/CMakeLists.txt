find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(ClipVolumeBox 
main.cpp 
LoadVolume.cpp
UserInterface.cpp
)

target_compile_definitions(ClipVolumeBox PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(ClipVolumeBox 
PUBLIC
${ITK_LIBRARIES}
userinterface
Eigen3::Eigen
)
