add_executable(test_util test_utils.cpp)

target_link_libraries(test_util PUBLIC
utils
Pal::Sigslot
)

#add_executable(test_asio test_asio.cpp)

#target_link_libraries(test_asio PUBLIC
#asio
#Pal::Sigslot
#utils
#OpenIGTLink
#)

add_executable(test_asio_2 )

target_link_libraries(test_asio_2 PUBLIC
asio
Pal::Sigslot
utils
OpenIGTLink
)