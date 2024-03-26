add_executable(child_with_gui 
child_with_gui.cpp 
)

target_compile_definitions(child_with_gui PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(child_with_gui PUBLIC
userinterface
communication
)

IF (WIN32)
  add_compile_definitions(child_with_gui CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(child_with_gui CURAN_LINUX)
ENDIF()

add_compile_definitions(child_with_gui CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(child_with_gui CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")

add_executable(parent_with_gui 
parent_with_gui.cpp 
)

target_compile_definitions(parent_with_gui PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(parent_with_gui PUBLIC
userinterface
communication
)

IF (WIN32)
  add_compile_definitions(parent_with_gui CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(parent_with_gui CURAN_LINUX)
ENDIF()

add_compile_definitions(parent_with_gui CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(parent_with_gui CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")
