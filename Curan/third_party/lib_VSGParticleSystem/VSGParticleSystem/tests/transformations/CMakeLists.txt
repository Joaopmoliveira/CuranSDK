set(transform_tests w2d d2f f2d d2n dynamics n2d_vel d2f_vel f2d_vel d2w_vel)

add_executable(checktransf checktransform.cpp TransformPoint.h TransformPoint.cpp)

target_link_libraries(
    checktransf
    PRIVATE
    vsgps
)

foreach(ttest ${transform_tests})
        add_test(
            NAME transform_test_${ttest}
            COMMAND checktransf ${ttest}
        )
endforeach(ttest ${transform_tests})

add_executable(checklimits checklimits.cpp)

target_link_libraries(checklimits PRIVATE vsgps)

add_test(
    NAME limits_test
    COMMAND checklimits
)