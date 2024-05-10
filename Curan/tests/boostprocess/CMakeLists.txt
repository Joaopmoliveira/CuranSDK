add_executable (test_boostprocess 
main.cpp 
)

target_include_directories(test_boostprocess PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(test_boostprocess ${Boost_LIBRARIES})
