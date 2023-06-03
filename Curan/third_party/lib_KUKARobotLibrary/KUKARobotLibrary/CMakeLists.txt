cmake_minimum_required(VERSION 3.11)

project(KukaRobotLibrary)

add_library (KukaRobotLibrary STATIC Robot.cpp RobotData.cpp  RobotModel.cpp ToolData.cpp)
#deleted library above: MatlabWrapper.cpp
if(TARGET rbdl-static)

else()
MESSAGE (ERROR "Could not find rbdl on your system.")
endif()

target_link_libraries(KukaRobotLibrary PUBLIC rbdl-static)
target_include_directories(KukaRobotLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
