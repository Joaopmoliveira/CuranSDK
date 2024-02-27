add_executable(
    checkdynamics3D
    checkdynamics3D.cpp
    Line.h
    DataTest.h
    DataTest3D.cpp
)
target_link_libraries(
    checkdynamics3D
    vsgps
    vsg::vsg 
    Eigen3::Eigen
)
add_test(
    NAME dynamics_test_3D
    COMMAND checkdynamics3D
)

add_executable(
    checkdynamics4D
    checkdynamics4D.cpp
    Line.h
    DataTest.h
    DataTest4D.cpp
)
target_link_libraries(
    checkdynamics4D
    vsgps
    vsg::vsg 
    Eigen3::Eigen
)
add_test(
    NAME dynamics_test_4D
    COMMAND checkdynamics4D
)

add_executable(
    checkdynamics5D
    checkdynamics5D.cpp
    Line.h
    DataTest.h
    DataTest5D.cpp
)
target_link_libraries(
    checkdynamics5D
    vsgps
    vsg::vsg 
    Eigen3::Eigen
)
add_test(
    NAME dynamics_test_5D
    COMMAND checkdynamics5D
)

add_executable(
    checkdynamics6D
    checkdynamics6D.cpp
    Line.h
    DataTest.h
    DataTest6D.cpp
)
target_link_libraries(
    checkdynamics6D
    vsgps
    vsg::vsg 
    Eigen3::Eigen
)
add_test(
    NAME dynamics_test_6D
    COMMAND checkdynamics6D
)
