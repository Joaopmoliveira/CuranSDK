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

add_executable(test_dummy_for_unittest_integration test_dummy_for_unittest_integration.cpp)

target_link_libraries(test_dummy_for_unittest_integration PUBLIC
communication
)

add_executable(test_fri_communication fri_openigtlink_dummy_implementation.cpp)

target_link_libraries(test_fri_communication PUBLIC
communication
)

add_executable(communication_with_arduino_or_esc32 communication_with_arduino_or_esc32.cpp)

target_link_libraries(communication_with_arduino_or_esc32 PUBLIC
asio
)

option(SERIAL_RECEIVE_ARGUMENT "This flag controls if the port is defined with an argument from the command line or a macro" FALSE)

if(SERIAL_RECEIVE_ARGUMENT)
    if(WIN32)
        target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="COM5")
    else(WIN32)
        target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="/dev/ttyUSB0")
    endif(WIN32)
else(SERIAL_RECEIVE_ARGUMENT)
    target_compile_definitions(communication_with_arduino_or_esc32 PRIVATE CURAN_SERIAL_PORT="COM5")
endif(SERIAL_RECEIVE_ARGUMENT)


add_executable(communication_with_ultrasound_arduino communication_with_arduino_or_esc32_ultrasound.cpp)

target_link_libraries(communication_with_ultrasound_arduino PUBLIC
asio
)

if(SERIAL_RECEIVE_ARGUMENT)
    if(WIN32)
        target_compile_definitions(communication_with_ultrasound_arduino PRIVATE CURAN_SERIAL_PORT="COM5")
    else(WIN32)
        target_compile_definitions(communication_with_ultrasound_arduino PRIVATE CURAN_SERIAL_PORT="/dev/ttyUSB0")
    endif(WIN32)
else(SERIAL_RECEIVE_ARGUMENT)
    target_compile_definitions(communication_with_ultrasound_arduino PRIVATE CURAN_SERIAL_PORT="COM5")
endif(SERIAL_RECEIVE_ARGUMENT)