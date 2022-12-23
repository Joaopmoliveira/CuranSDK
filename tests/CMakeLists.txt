add_executable(test_util test_utils.cpp)

target_link_libraries(test_util PUBLIC
utils
)

add_executable(test_asio test_asio.cpp)

target_link_libraries(test_asio PUBLIC
asio
utils
)