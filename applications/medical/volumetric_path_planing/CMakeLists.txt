find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(VolumetricPathPlanning 
main.cpp 
BoundingBox.cpp
LoadVolume.cpp
UserInterface.cpp
)

target_compile_definitions(VolumetricPathPlanning PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(VolumetricPathPlanning 
PUBLIC
${ITK_LIBRARIES}
userinterface
Eigen3::Eigen
)
