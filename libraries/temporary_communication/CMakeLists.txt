add_library(communication STATIC
src/ChannelManager.cpp
src/Functor.cpp
src/ProcessorOIGTL.cpp
 "include/communication/ServerManager.h" "include/communication/ClientManager.h" "src/Server.cpp" "include/communication/Protocol.h")

target_include_directories(communication
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(communication PUBLIC
OpenIGTLink
Pal::Sigslot
asio::asio
utils
)