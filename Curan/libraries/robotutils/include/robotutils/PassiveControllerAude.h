#ifndef CURAN_PASSIVE_CONTROLLER_AUDE_
#define CURAN_PASSIVE_CONTROLLER_AUDE_        

#include "LBRController.h"
#include "gaussianmixtures/GMR.h"

namespace curan {
namespace robotic {

struct PassiveControllerData : public UserData{
    PassiveControllerData(const std::string& model_file);

    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state) override;

    curan::gaussian::GMR<3,3> model;
};

}
}

#endif