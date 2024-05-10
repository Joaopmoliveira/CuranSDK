find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(ApplicationLauncher
main.cpp
)

target_compile_definitions(ApplicationLauncher PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
add_compile_definitions(ApplicationLauncher CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(ApplicationLauncher CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")

target_include_directories(ApplicationLauncher PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(ApplicationLauncher PUBLIC
userinterface
processmanagement
${ITK_LIBRARIES}
${Boost_LIBRARIES}
)

