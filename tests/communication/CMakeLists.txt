add_executable(test_asio test_asio.cpp)

target_link_libraries(test_asio PUBLIC
asio
utils
OpenIGTLink
)

IF (WIN32)
  add_compile_definitions(utils WIN32_LEAN_AND_MEAN)
ENDIF()

add_executable(test_communication test_communication.cpp)

target_link_libraries(test_communication PUBLIC
communication
)