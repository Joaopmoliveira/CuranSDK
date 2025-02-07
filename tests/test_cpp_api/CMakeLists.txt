add_executable(cpp_generic_api_testing main.cpp)

target_link_libraries(cpp_generic_api_testing PUBLIC 
utils
)

add_executable(test_variant_callback_implementation test_variant_storage_api.cpp)

add_executable(test_new_capture_buffer_arquitecture test_new_capture_buffer_arquitecture.cpp)

target_link_libraries(test_new_capture_buffer_arquitecture PUBLIC 
utils
)

find_package(pugixml CONFIG REQUIRED)

add_executable(test_std_cin_limitations test_std_cin_limitations.cpp)

target_link_libraries(test_std_cin_limitations PUBLIC 
pugixml::static 
pugixml::pugixml
)

add_executable(test_cpp_laucher test_cpp_laucher.cpp)

IF (WIN32)
  add_compile_definitions(test_cpp_laucher CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(test_cpp_laucher CURAN_LINUX)
ENDIF()

add_executable(test_dummy test_secondary_dummy_project.cpp)

target_link_libraries(test_dummy PUBLIC 
utils
)

add_executable(test_chrono_duration_propagation test_chrono_duration_propagation.cpp)

add_executable(test_variadic_string_command test_variadic_string_command.cpp)

add_executable(test_executable_name_injection test_executable_name_injection.cpp)

add_compile_definitions(test_executable_name_injection CURAN_BINARY_LOCATION="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
add_compile_definitions(test_executable_name_injection CURAN_BINARY_SUFFIX="${CMAKE_EXECUTABLE_SUFFIX}")

add_executable(test_geometry_computation test_geometry_computation.cpp)

target_link_libraries(test_geometry_computation PUBLIC 
GeometricTools
)

add_executable(test_process_lauch_with_asio test_process_lauch_with_asio.cpp)

target_include_directories(test_process_lauch_with_asio PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(test_process_lauch_with_asio PUBLIC 
${Boost_LIBRARIES}
)

