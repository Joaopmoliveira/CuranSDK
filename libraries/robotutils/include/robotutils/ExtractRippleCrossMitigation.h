#ifndef CURAN_EXTRACT_RIPPLE_CROSS_MITIGATION
#define CURAN_EXTRACT_RIPPLE_CROSS_MITIGATION

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

struct ExtractRippleCrossMitigation : public UserData{
    ExtractRippleCrossMitigation();
    std::array<FilterData,number_of_joints> joint_data_first_harmonic;
    std::array<FilterData,number_of_joints> joint_data_second_harmonic;
    std::array<FilterProperties,number_of_joints> first_harmonic;
    std::array<FilterProperties,number_of_joints> second_harmonic;
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif