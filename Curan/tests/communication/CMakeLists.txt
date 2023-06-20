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

add_executable(communication_with_arduino_or_esc32 communication_with_arduino_or_esc32.cpp)

target_link_libraries(communication_with_arduino_or_esc32 PUBLIC
asio
)

option(SERIAL_RECEIVE_ARGUMENT "This flag controls if the port is defined with an argument from the command line or a macro" false)

if(SERIAL_RECEIVE_ARGUMENT)
    if(WIN32)
        target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="COM1")
    else(WIN32)
        target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="/dev/ttyUSB0")
    endif(WIN32)
else(SERIAL_RECEIVE_ARGUMENT)
    target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="")
endif(SERIAL_RECEIVE_ARGUMENT)
