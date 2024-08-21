add_library(communicationtemplated STATIC
src/SerialClient.cpp
)

target_include_directories(communicationtemplated
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(communicationtemplated PUBLIC
OpenIGTLink
utils
)