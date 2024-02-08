#include "robotutils/HandGuidance.h"

namespace curan {
namespace robotic {

EigenState handguidance(void* user_pointer,kuka::Robot* robot, RobotParameters* iiwa, EigenState state){
    static double currentTime = 0.0;
    state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;
    state.cmd_q = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
    currentTime += state.sampleTime;
    return state;
};

}
}