find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(Registration
main.cpp
)

target_compile_definitions(Registration PRIVATE vsgXchange_FOUND)
target_compile_definitions(Registration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(Registration
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)