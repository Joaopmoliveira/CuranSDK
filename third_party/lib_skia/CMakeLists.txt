cmake_minimum_required(VERSION 3.8)

##
## PROJECT
## name and version
##
project(skia VERSION 1.0)

if(NOT EXISTS "${SKIA_BINARY_DIR}" OR NOT EXISTS "${SKIA_SOURCE_DIR}" )
	message(FATAL_ERROR "Could not find skia library. Specify the variable SKIA_BINARY_DIR with the location of the library binary files (i.e. skia/out/Static) and the SKIA_SOURCE_DIR which corresponds to the path where skia source code is located")
endif()

add_library(${PROJECT_NAME} STATIC IMPORTED GLOBAL)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}  PROPERTIES 
	IMPORTED_LOCATION 	${SKIA_BINARY_DIR}/skia.lib 
	INTERFACE 	${SKIA_SOURCE_DIR})