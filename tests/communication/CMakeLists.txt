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

add_executable(test_proc_handler_communication proc_handling_communication.cpp)

target_link_libraries(test_proc_handler_communication PUBLIC
communication
)

add_executable(mimic_proc_handling_multiprocess mimic_proc_handling_multiprocess.cpp)

target_link_libraries(mimic_proc_handling_multiprocess PUBLIC
communication
utils
)

add_executable(classes_proc_handling classes_proc_handling.cpp)

target_link_libraries(classes_proc_handling PUBLIC
communication
utils
)

add_executable(mimic_child_proc mimic_child_proc.cpp)

target_link_libraries(mimic_child_proc PUBLIC
communication
utils
)

add_executable(mimic_parent_proc mimic_parent_proc.cpp)

target_link_libraries(mimic_parent_proc PUBLIC
communication
utils
)

IF (WIN32)
  add_compile_definitions(mimic_parent_proc CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(mimic_parent_proc CURAN_LINUX)
ENDIF()

add_compile_definitions(mimic_parent_proc CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(mimic_parent_proc CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")