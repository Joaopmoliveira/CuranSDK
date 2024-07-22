add_library(communication STATIC
src/Client.cpp 
src/ProtoIGTL.cpp	
src/ProtoFRI.cpp
src/ProtoProcHandler.cpp
src/Server.cpp	
src/Socket.cpp	
src/Protocols.cpp
src/SerialClient.cpp
)

target_include_directories(communication
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(communication PUBLIC
OpenIGTLink
utils
)