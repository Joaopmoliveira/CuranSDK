include(GoogleTest)

add_executable(StringManipulation string_manipulation.cpp)
target_link_libraries(StringManipulation GTest::gtest_main utils)
gtest_discover_tests(StringManipulation)

add_executable(SafeQueue safe_queue.cpp)
target_link_libraries(SafeQueue GTest::gtest_main utils)
gtest_discover_tests(SafeQueue)

add_executable(Reader reader.cpp)
target_link_libraries(Reader GTest::gtest_main utils)
gtest_discover_tests(Reader)

add_executable(MemoryUtils memory_utils.cpp)
target_link_libraries(MemoryUtils GTest::gtest_main utils)
gtest_discover_tests(MemoryUtils)

add_executable(DateFormater date_formater.cpp)
target_link_libraries(DateFormater GTest::gtest_main utils)
gtest_discover_tests(DateFormater)

add_executable(FileSharing file_data.cpp)
target_link_libraries(FileSharing GTest::gtest_main utils)
gtest_discover_tests(FileSharing)