add_library(utils STATIC
src/Flag.cpp
src/Job.cpp
src/Logger.cpp
src/TheadPool.cpp
src/MemoryUtils.cpp
src/Cancelable.cpp
)

target_include_directories(utils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(utils PUBLIC
asio
)

add_dependencies(utils WIDGETRESOURCES)
