find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})


add_executable(ApplicationLauncher
main.cpp
)

if(WAS_PLUS_SPECIFIED)
    target_compile_definitions(ApplicationLauncher PRIVATE
    CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}"
    CURAN_PLUS_EXECUTABLE_PATH="${CURAN_PLUS_EXECUTABLE_PATH}")
else()
    target_compile_definitions(ApplicationLauncher PRIVATE
    CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
endif()


add_compile_definitions(ApplicationLauncher CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(ApplicationLauncher CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")

target_include_directories(ApplicationLauncher PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(ApplicationLauncher PUBLIC
userinterface
${ITK_LIBRARIES}
${Boost_LIBRARIES}
)

