add_library(communication STATIC
src/SerialClient.cpp
src/ProtoFRI.cpp
src/ProtoIGTL.cpp
src/ProtoCommand.cpp
)

target_include_directories(communication
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(communication PUBLIC
OpenIGTLink
utils
)