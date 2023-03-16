add_executable(test_asio test_asio.cpp)

target_link_libraries(test_asio PUBLIC
asio
utils
OpenIGTLink
)

add_executable(test_communication test_asio.cpp)

target_link_libraries(test_communication PUBLIC
communication
)