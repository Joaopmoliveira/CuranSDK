#ifndef CURAN_LIMIT_CYCLE_CONTROLLER_AUDE_
#define CURAN_LIMIT_CYCLE_CONTROLLER_AUDE_

#include "LBRController.h"
#include "gaussianmixtures/GMR.h"

namespace curan {
namespace robotic {

struct Transformation{
    Eigen::Matrix<double,3,3> rotation;
    Eigen::Matrix<double,3,1> translation;
};


struct LimitCycleController : public UserData{
    LimitCycleController(const std::string& model_file,const std::string& transform_file);

    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    curan::gaussian::GMR<2,2> model;
    Transformation transformation_to_model_coordinates;
};

}
}

#endif