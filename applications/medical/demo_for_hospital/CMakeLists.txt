find_package(ITK CONFIG REQUIRED)     

include(${ITK_USE_FILE})

add_executable(DemoSurgicalPlanning
main.cpp 
)

target_compile_definitions(DemoSurgicalPlanning PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(DemoSurgicalPlanning 
PUBLIC
Eigen3::Eigen
${ITK_LIBRARIES}
userinterface
geometry
)
