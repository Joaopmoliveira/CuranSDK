cmake_minimum_required(VERSION 3.8)

##
## PROJECT
## name and version
##
project(asio VERSION 1.0)

set(ASIO_BUILD_INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(
	${PROJECT_NAME} 
	INTERFACE
	$<BUILD_INTERFACE:${ASIO_BUILD_INTERFACE}>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

#installation

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

export(
    TARGETS ${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE asio::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)