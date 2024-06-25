add_library(utils STATIC
src/Job.cpp
src/Logger.cpp
src/TheadPool.cpp
src/MemoryUtils.cpp
src/Reader.cpp
src/ModifyXMLField.cpp
)

target_include_directories(utils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(utils PUBLIC
asio
pugixml::static 
pugixml::pugixml
)

IF (WIN32)
  target_compile_definitions(utils PUBLIC CURAN_WINDOWS)
  target_compile_definitions(utils PUBLIC NOMINMAX)
ELSEIF(UNIX)
  target_compile_definitions(utils PUBLIC CURAN_LINUX)
ENDIF()

add_dependencies(utils 
curan_resources
Eigen3::Eigen
)
