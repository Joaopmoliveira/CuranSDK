if(NOT EXISTS "${SKIA_BINARY_DIR}" OR NOT EXISTS "${SKIA_SOURCE_DIR}" )
	message(FATAL_ERROR "Could not find skia library. Specify the variable SKIA_BINARY_DIR with the location of the library binary files (i.e. skia/out/Static) and the SKIA_SOURCE_DIR which corresponds to the path where skia source code is located")
else()
	add_library(skia STATIC IMPORTED)
	set_target_properties(skia PROPERTIES 
		IMPORTED_LOCATION 	${SKIA_BINARY_DIR}/skia.lib 
		INTERFACE 	${SKIA_SOURCE_DIR})
endif()