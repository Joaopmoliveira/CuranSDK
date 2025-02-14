#ifndef CURAN_VIEW_FILTERING_
#define CURAN_VIEW_FILTERING_

#include "LBRController.h"
#include "RippleFilter.h"
#include "GenericStateDerivative.h"
#include <array>

namespace curan {
namespace robotic {

struct ViewFiltering : public UserData{
    std::array<ripple::Data, number_of_joints> joint_data_first_harmonic;
    std::array<ripple::Data, number_of_joints> joint_data_second_harmonic;
    std::array<ripple::Data, number_of_joints> joint_data_third_harmonic;
    std::array<ripple::Properties, number_of_joints> first_harmonic;
    std::array<ripple::Properties, number_of_joints> second_harmonic;
    std::array<ripple::Properties, number_of_joints> third_harmonic;

    LowPassDerivativeFilter<number_of_joints> filtering_mechanism;
    Eigen::Matrix<double,number_of_joints,1> previous_q;

    bool is_first_loop = true;
    ViewFiltering();
    virtual ~ViewFiltering(){};
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif