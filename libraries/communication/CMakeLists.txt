add_library(communication
src/ChannelManager.cpp
src/Functor.cpp
src/ProcessorOIGTL.cpp
)

target_include_directories(communication
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(communication PUBLIC
OpenIGTLink
Pal::Sigslot
asio::asio
)