add_executable(forcelimit force_limit.cpp)

target_link_libraries(
    forcelimit
    PRIVATE
    vsgps
    vsg::vsg
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

add_executable(rodrigues rodrigues_formula.cpp)

target_link_libraries(
    rodrigues
    PRIVATE
    vsg::vsg
)

add_executable(polytransform poly_transform.cpp)

add_executable(optiontransform option_transform.cpp)

target_link_libraries(
    optiontransform
    PRIVATE
    vsgps
    vsg::vsg
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

add_executable(wireframecube wireframeCube.cpp)

target_link_libraries(
    wireframecube
    PRIVATE
    vsgps
)

add_executable(logtransform log_transformations.cpp)

target_link_libraries(
    logtransform
    PRIVATE
    vsgps
)

add_executable(logstep logging_step.cpp)

target_link_libraries(
    logstep
    PRIVATE
    vsgps
)

if (vsgXchange_FOUND)
    target_compile_definitions(rodrigues PRIVATE vsgXchange_FOUND)
    target_link_libraries(rodrigues PRIVATE vsgXchange::vsgXchange)

    target_compile_definitions(forcelimit PRIVATE vsgXchange_FOUND)
    target_link_libraries(forcelimit PRIVATE vsgXchange::vsgXchange)

    target_compile_definitions(optiontransform PRIVATE vsgXchange_FOUND)
    target_link_libraries(optiontransform PRIVATE vsgXchange::vsgXchange)

    target_compile_definitions(wireframecube PRIVATE vsgXchange_FOUND)
    target_link_libraries(wireframecube PRIVATE vsgXchange::vsgXchange)
endif()

install(TARGETS rodrigues
        RUNTIME DESTINATION bin
)


install(TARGETS polytransform 
        RUNTIME DESTINATION bin
)

install(TARGETS forcelimit 
        RUNTIME DESTINATION bin
)

install(TARGETS optiontransform 
        RUNTIME DESTINATION bin
)

install(TARGETS wireframecube 
        RUNTIME DESTINATION bin
)

install(TARGETS logtransform 
        RUNTIME DESTINATION bin
)

install(TARGETS logstep 
        RUNTIME DESTINATION bin
)
