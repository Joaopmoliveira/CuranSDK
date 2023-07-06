find_package(vsg)
find_package(vsgXchange)
find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(test_plus_playback main.cpp)

target_compile_definitions(test_plus_playback PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_plus_playback PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_plus_playback PUBLIC
communication
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
)
