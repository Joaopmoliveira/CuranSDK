add_library(processmanagement STATIC
src/ChildProcess.cpp
src/SingleChildParent.cpp
)


target_include_directories(processmanagement
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(processmanagement PUBLIC
communication
)

IF (WIN32)
  add_compile_definitions(processmanagement CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(processmanagement CURAN_LINUX)
ENDIF()