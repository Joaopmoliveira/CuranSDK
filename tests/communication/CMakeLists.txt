add_executable(test_asio test_asio.cpp)

target_link_libraries(test_asio PUBLIC
asio
utils
OpenIGTLink
)

add_executable(test_communication test_communication.cpp)

target_link_libraries(test_communication PUBLIC
communication
)

add_executable(test_OpenIGTLink test_openIGTlink.cpp)

target_link_libraries(test_OpenIGTLink PUBLIC
OpenIGTLink
)