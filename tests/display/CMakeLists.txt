add_executable(test_user_interface test_user_interface.cpp)

target_link_libraries(test_user_interface PUBLIC
utils
userinterface
)

add_executable(test_library_userinterface test_library_userinterface.cpp)

target_link_libraries(test_library_userinterface PUBLIC
userinterface
)

add_executable(test_button test_button.cpp)

target_link_libraries(test_button PUBLIC
userinterface
)
