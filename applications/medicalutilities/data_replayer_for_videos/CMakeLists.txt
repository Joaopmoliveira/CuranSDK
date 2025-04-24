add_executable(DataReplayer
main.cpp
)

target_compile_definitions(DataReplayer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}" CURAN_PLUS_EXECUTABLE_PATH="${CURAN_PLUS_EXECUTABLE_PATH}")

add_compile_definitions(DataReplayer CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(DataReplayer CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")

target_include_directories(DataReplayer PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(DataReplayer PUBLIC
Eigen3::Eigen
robotutils
utils
${OPERATING_SYSTEM_REQUIRMENTS}
communication
${Boost_LIBRARIES}
)

