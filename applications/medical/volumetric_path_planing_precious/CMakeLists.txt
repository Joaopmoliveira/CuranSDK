find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(VolumetricPathPlanningPrecious
main.cpp 
BoundingBox.cpp
UserInterface.cpp
)

target_compile_definitions(VolumetricPathPlanningPrecious PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(VolumetricPathPlanningPrecious 
PUBLIC
${ITK_LIBRARIES}
userinterface
Eigen3::Eigen
)
