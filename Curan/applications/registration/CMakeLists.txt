find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(robotic_registration
main.cpp
)

target_compile_definitions(robotic_registration PRIVATE vsgXchange_FOUND)
target_compile_definitions(robotic_registration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(robotic_registration
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
)