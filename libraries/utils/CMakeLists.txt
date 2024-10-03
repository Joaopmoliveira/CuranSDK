add_library(utils STATIC
src/Job.cpp
src/Logger.cpp
src/TheadPool.cpp
src/MemoryUtils.cpp
src/Reader.cpp
src/ModifyXMLField.cpp
src/FileStructures.cpp
)

target_include_directories(utils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(utils PUBLIC
asio
pugixml::static 
pugixml::pugixml
nlohmann_json 
nlohmann_json::nlohmann_json 
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
