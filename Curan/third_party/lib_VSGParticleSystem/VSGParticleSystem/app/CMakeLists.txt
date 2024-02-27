add_executable(vsgparticlesystem main.cpp)

target_link_libraries(
    vsgparticlesystem
    PRIVATE
    vsgps 
    vsg::vsg
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

add_executable(simtrajs simulate_trajectories.cpp)

target_link_libraries(
    simtrajs
    PRIVATE
    vsgps 
    vsg::vsg
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

if (vsgXchange_FOUND)
    target_compile_definitions(vsgparticlesystem PRIVATE vsgXchange_FOUND)
    target_link_libraries(vsgparticlesystem PRIVATE vsgXchange::vsgXchange)

    target_compile_definitions(simtrajs PRIVATE vsgXchange_FOUND)
    target_link_libraries(simtrajs PRIVATE vsgXchange::vsgXchange)
endif()

install(TARGETS vsgparticlesystem
        RUNTIME DESTINATION bin
)

install(TARGETS simtrajs
        RUNTIME DESTINATION bin
)

add_subdirectory(demos)