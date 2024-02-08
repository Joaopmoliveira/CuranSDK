cmake_minimum_required(VERSION 3.11)

project(tasksLib)

add_library (tasksLib STATIC jlA.cpp tasks.cpp singAvoid.cpp taskCreator.cpp vwA.cpp obA.cpp )
target_include_directories(tasksLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(tasksLib PUBLIC rbdl-static robotParameters)

